#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lekiwi_base_controller/base_controller.hpp"
#include "lekiwi_base_controller/odometry.hpp"
#include "lekiwi_base_controller/omni_kinematics.hpp"

namespace
{

std::string ReadSourceFile(const std::string & relative_path)
{
  std::ifstream stream(std::string(LEKIWI_BASE_CONTROLLER_SOURCE_DIR) + "/" + relative_path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

}  // namespace

TEST(BaseControllerBuildContractTest, DeclaresStdMsgsAndLekiwiMsgsDependencies)
{
  const auto cmake = ReadSourceFile("CMakeLists.txt");
  const auto package_xml = ReadSourceFile("package.xml");

  EXPECT_NE(cmake.find("find_package(std_msgs REQUIRED)"), std::string::npos);
  EXPECT_NE(cmake.find("  std_msgs\n"), std::string::npos);
  EXPECT_NE(package_xml.find("<depend>std_msgs</depend>"), std::string::npos);
  EXPECT_NE(cmake.find("find_package(lekiwi_msgs REQUIRED)"), std::string::npos);
  EXPECT_NE(package_xml.find("<depend>lekiwi_msgs</depend>"), std::string::npos);
}

TEST(BaseControllerBuildContractTest, DescriptionMakesNoHardwareAcceptanceClaim)
{
  const auto package_xml = ReadSourceFile("package.xml");

  EXPECT_NE(package_xml.find("<description>"), std::string::npos);
  EXPECT_EQ(package_xml.find("skeleton"), std::string::npos);
  EXPECT_EQ(package_xml.find("hardware-accepted"), std::string::npos);
}

TEST(BaseControllerPublisherContractTest, MotorReadyUsesTransientLocalReliableQos)
{
  const auto node_source = ReadSourceFile("src/lekiwi_base_controller_node.cpp");
  const auto coordinator_source = ReadSourceFile("src/motor_safety_coordinator.cpp");

  EXPECT_NE(
    coordinator_source.find("rclcpp::QoS(1).reliable().transient_local()"),
    std::string::npos);
  EXPECT_NE(node_source.find("auto qos = MakeMotorReadyQos()"), std::string::npos);
  EXPECT_NE(
    node_source.find(
      "create_publisher<std_msgs::msg::Bool>(params_.motor_ready_topic, qos)"),
    std::string::npos);
}

// 버스는 lekiwi_motor_driver_node만 연다. 이 패키지는 시리얼 포트를 열지 않는다
TEST(BaseControllerSerialContractTest, NeverOpensSerialPort)
{
  for (const char * path : {
      "src/base_controller.cpp", "src/driver_motor_interface.cpp",
      "src/lekiwi_base_controller_node.cpp", "src/lekiwi_base_controller_main.cpp",
      "src/motor_safety_coordinator.cpp", "src/wheel_feedback_estimator.cpp",
      "include/lekiwi_base_controller/driver_motor_interface.hpp",
      "config/lekiwi_base_controller.yaml"})
  {
    const auto source = ReadSourceFile(path);
    ASSERT_FALSE(source.empty()) << path;
    EXPECT_EQ(source.find("termios"), std::string::npos) << path;
    EXPECT_EQ(source.find("fcntl"), std::string::npos) << path;
    EXPECT_EQ(source.find("/dev/tty"), std::string::npos) << path;
    EXPECT_EQ(source.find("serial_port"), std::string::npos) << path;
  }
}

TEST(BaseControllerSafetyWiringContractTest, UsesCoordinatorForCycleServiceAndPublication)
{
  const auto source = ReadSourceFile("src/lekiwi_base_controller_node.cpp");

  EXPECT_NE(source.find("motor_safety_->BeforeCommand(current_time)"), std::string::npos);
  EXPECT_NE(source.find("motor_safety_->SendWheelCommand(command_wheel_speeds)"), std::string::npos);
  EXPECT_NE(
    source.find("motor_safety_->GetFreshEncoderWheelSpeeds(safety_time)"), std::string::npos);
  EXPECT_NE(source.find("motor_safety_->SetMotorPower(request->data, now())"), std::string::npos);
  EXPECT_NE(source.find("message.data = motor_safety_->IsMotorReady()"), std::string::npos);
}

TEST(BaseControllerSafetyWiringContractTest, ControlCallbacksShareOneMutuallyExclusiveGroup)
{
  const auto node_source = ReadSourceFile("src/lekiwi_base_controller_node.cpp");
  const auto main_source = ReadSourceFile("src/lekiwi_base_controller_main.cpp");
  const auto driver_source = ReadSourceFile("src/driver_motor_interface.cpp");

  EXPECT_NE(
    node_source.find(
      "control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)"),
    std::string::npos);
  EXPECT_NE(node_source.find("cmd_vel_options.callback_group = control_group_"), std::string::npos);
  EXPECT_NE(node_source.find("[this]() {ControlTimerCallback();}, control_group_)"), std::string::npos);
  EXPECT_NE(main_source.find("rclcpp::executors::MultiThreadedExecutor executor("), std::string::npos);
  EXPECT_NE(
    driver_source.find("config_.wheel_torque_service, rclcpp::ServicesQoS(), torque_client_group_)"),
    std::string::npos);
  EXPECT_NE(driver_source.find("states_options.callback_group = states_group_"), std::string::npos);
}

TEST(OmniKinematicsTest, ZeroTwistProducesStoppedWheelCommands)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  const auto command = kinematics.ToWheelSpeeds(0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(command.left, 0.0);
  EXPECT_DOUBLE_EQ(command.back, 0.0);
  EXPECT_DOUBLE_EQ(command.right, 0.0);
}

TEST(BaseControllerBackendTest, DriverWithoutMotorWriteUsesMockLoggingBackend)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = false;

  EXPECT_EQ(
    lekiwi_base_controller::BaseMotorBackendKind::MockLogging,
    lekiwi_base_controller::SelectBaseMotorBackend(params));
}

TEST(BaseControllerBackendTest, DriverWithMotorWriteUsesDriverHardwareBackend)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;

  EXPECT_EQ(
    lekiwi_base_controller::BaseMotorBackendKind::DriverHardware,
    lekiwi_base_controller::SelectBaseMotorBackend(params));
}

TEST(BaseControllerCommandLimitTest, RejectsNonFiniteTwistAsZero)
{
  const lekiwi_base_controller::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  geometry_msgs::msg::Twist command;
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  command.angular.z = 0.2;
  const auto limited = lekiwi_base_controller::LimitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);
  EXPECT_DOUBLE_EQ(limited.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(limited.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(limited.angular.z, 0.0);
}

TEST(BaseControllerCommandLimitTest, PreservesDirectionWhileApplyingAxisAndPlanarLimits)
{
  const lekiwi_base_controller::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.52;
  command.linear.y = 0.20;
  const auto limited = lekiwi_base_controller::LimitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);
  EXPECT_LE(std::abs(limited.linear.x), 0.26);
  EXPECT_LE(std::abs(limited.linear.y), 0.15);
  EXPECT_LE(std::hypot(limited.linear.x, limited.linear.y), 0.26 + 1e-12);
  EXPECT_NEAR(limited.linear.x / limited.linear.y, command.linear.x / command.linear.y, 1e-12);
  EXPECT_DOUBLE_EQ(limited.angular.z, 0.0);
}

TEST(BaseControllerCommandLimitTest, ClampsWideEnvelopeAngularLimit)
{
  const lekiwi_base_controller::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  geometry_msgs::msg::Twist command;
  command.angular.z = 3.64;

  const auto limited = lekiwi_base_controller::LimitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);

  EXPECT_NEAR(limited.angular.z, 1.82, 1e-12);
}

TEST(BaseControllerCommandLimitTest, CombinedBodyCommandUsesUniformWheelScale)
{
  constexpr double kWheelRadius = 0.0508;
  constexpr double kBaseRadius = 0.115;
  constexpr double kMaxWheelSpeed = 5.4;
  const lekiwi_base_controller::OmniKinematics kinematics(
    kWheelRadius, kBaseRadius, kMaxWheelSpeed);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.26 * std::sqrt(3.0) * 0.5;
  command.linear.y = 0.26 * 0.5;
  command.angular.z = 1.82;
  const double expected_peak_wheel_speed =
    (std::sqrt(3.0) * 0.5 * command.linear.x + 0.5 * command.linear.y +
    kBaseRadius * command.angular.z) / kWheelRadius;
  const double expected_scale = kMaxWheelSpeed / expected_peak_wheel_speed;

  const auto limited = lekiwi_base_controller::LimitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);

  ASSERT_LT(expected_scale, 1.0);
  EXPECT_NEAR(limited.linear.x, command.linear.x * expected_scale, 1e-12);
  EXPECT_NEAR(limited.linear.y, command.linear.y * expected_scale, 1e-12);
  EXPECT_NEAR(limited.angular.z, command.angular.z * expected_scale, 1e-12);

  const auto odom_twist = lekiwi_base_controller::SelectOdometryUpdateTwist(
    limited, std::nullopt, "command", kinematics);
  EXPECT_NEAR(odom_twist.linear.x, limited.linear.x, 1e-12);
  EXPECT_NEAR(odom_twist.linear.y, limited.linear.y, 1e-12);
  EXPECT_NEAR(odom_twist.angular.z, limited.angular.z, 1e-12);
}

TEST(BaseControllerOdometryTest, TimeoutCommandProducesZeroOdomTwist)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto active_twist =
    lekiwi_base_controller::SelectOdometryTwist(command, true, 0.29, 0.3);
  EXPECT_DOUBLE_EQ(active_twist.linear.x, command.linear.x);
  EXPECT_DOUBLE_EQ(active_twist.linear.y, command.linear.y);
  EXPECT_DOUBLE_EQ(active_twist.angular.z, command.angular.z);

  const auto timeout_twist =
    lekiwi_base_controller::SelectOdometryTwist(command, true, 0.31, 0.3);
  EXPECT_DOUBLE_EQ(timeout_twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(timeout_twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(timeout_twist.angular.z, 0.0);
}

TEST(BaseControllerOdometrySourceTest, CommandSourceUsesCommandTwist)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto selected = lekiwi_base_controller::SelectOdometryUpdateTwist(
    command, lekiwi_base_controller::WheelCommand{1.0, 2.0, 3.0}, "command", kinematics);

  EXPECT_DOUBLE_EQ(selected.linear.x, command.linear.x);
  EXPECT_DOUBLE_EQ(selected.linear.y, command.linear.y);
  EXPECT_DOUBLE_EQ(selected.angular.z, command.angular.z);
}

TEST(BaseControllerOdometrySourceTest, EncoderSourceUsesBodyTwistFromWheelSpeeds)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;
  const auto wheel_speeds = kinematics.ToWheelSpeeds(0.12, -0.04, 0.2);

  const auto selected = lekiwi_base_controller::SelectOdometryUpdateTwist(
    command, wheel_speeds, "encoder", kinematics);
  const auto expected = kinematics.ToBodyTwist(wheel_speeds);

  EXPECT_NEAR(selected.linear.x, expected.linear_x, 1e-12);
  EXPECT_NEAR(selected.linear.y, expected.linear_y, 1e-12);
  EXPECT_NEAR(selected.angular.z, expected.angular_z, 1e-12);
}

TEST(BaseControllerOdometrySourceTest, EncoderSourceReadFailureReturnsZeroTwist)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto selected = lekiwi_base_controller::SelectOdometryUpdateTwist(
    command, std::nullopt, "encoder", kinematics);

  EXPECT_DOUBLE_EQ(selected.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(selected.angular.z, 0.0);
}

TEST(BaseControllerOdometrySourceTest, EncoderDeltaChangesPoseOnlyOnItsReadCycle)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  lekiwi_base_controller::Odometry odometry;
  const lekiwi_base_controller::WheelPositionDelta wheel_delta{0.1, -0.2, 0.3};
  geometry_msgs::msg::Twist smoothed_twist;
  smoothed_twist.linear.x = 0.7;
  smoothed_twist.linear.y = -0.4;
  smoothed_twist.angular.z = 0.6;

  lekiwi_base_controller::UpdateOdometryForSource(
    odometry, "encoder", "position", smoothed_twist, 0.02, wheel_delta, kinematics);
  const auto after_delta = odometry.ToMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", smoothed_twist);

  for (int cycle = 0; cycle < 2; ++cycle) {
    lekiwi_base_controller::UpdateOdometryForSource(
      odometry, "encoder", "position", smoothed_twist, 0.02, std::nullopt, kinematics);
  }
  const auto after_holds = odometry.ToMessage(
    rclcpp::Time(0, 40000000), "odom", "base_footprint", smoothed_twist);

  EXPECT_DOUBLE_EQ(after_holds.pose.pose.position.x, after_delta.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(after_holds.pose.pose.position.y, after_delta.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(after_holds.pose.pose.orientation.z, after_delta.pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(after_holds.pose.pose.orientation.w, after_delta.pose.pose.orientation.w);
  EXPECT_DOUBLE_EQ(after_holds.twist.twist.linear.x, smoothed_twist.linear.x);
  EXPECT_DOUBLE_EQ(after_holds.twist.twist.linear.y, smoothed_twist.linear.y);
  EXPECT_DOUBLE_EQ(after_holds.twist.twist.angular.z, smoothed_twist.angular.z);
}

TEST(BaseControllerOdometrySourceTest, SpeedFeedbackEncoderSourceIntegratesWheelDerivedTwist)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  lekiwi_base_controller::Odometry odometry;
  geometry_msgs::msg::Twist encoder_twist;
  encoder_twist.linear.x = 0.12;
  encoder_twist.linear.y = -0.04;
  encoder_twist.angular.z = 0.2;

  lekiwi_base_controller::UpdateOdometryForSource(
    odometry, "encoder", "speed", encoder_twist, 0.5, std::nullopt, kinematics);
  const auto msg = odometry.ToMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", encoder_twist);

  EXPECT_NEAR(msg.pose.pose.position.x, 0.06, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, -0.02, 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(0.1 / 2.0), 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(0.1 / 2.0), 1e-12);
}

TEST(BaseControllerOdometrySourceTest, EncoderPoseUsesRawNetTickDeltaNotSmoothedEventSum)
{
  constexpr double kTicksPerRevolution = 4096.0;
  constexpr double kPi = 3.14159265358979323846;
  const lekiwi_base_controller::OmniKinematics kinematics;
  lekiwi_base_controller::Odometry odometry;
  const double angle_per_tick = 2.0 * kPi / kTicksPerRevolution;
  const lekiwi_base_controller::WheelPositionDelta raw_net_delta{-5.0 * angle_per_tick, 0.0, 0.0};
  const auto expected = kinematics.ToBodyDelta(raw_net_delta);
  geometry_msgs::msg::Twist larger_smoothed_window_twist;
  larger_smoothed_window_twist.linear.x = 8.0;
  larger_smoothed_window_twist.linear.y = -6.0;
  larger_smoothed_window_twist.angular.z = 4.0;

  lekiwi_base_controller::UpdateOdometryForSource(
    odometry,
    "encoder",
    "position",
    larger_smoothed_window_twist,
    0.02,
    raw_net_delta,
    kinematics);
  const auto msg = odometry.ToMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", larger_smoothed_window_twist);

  EXPECT_NEAR(msg.pose.pose.position.x, expected.dx, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, expected.dy, 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(expected.dyaw / 2.0), 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(expected.dyaw / 2.0), 1e-12);
  EXPECT_DOUBLE_EQ(msg.twist.twist.linear.x, larger_smoothed_window_twist.linear.x);
}

TEST(BaseControllerOdometrySourceTest, CommandSourceIntegratesTwistAndIgnoresEncoderDelta)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  lekiwi_base_controller::Odometry odometry;
  geometry_msgs::msg::Twist command_twist;
  command_twist.linear.x = 0.2;
  command_twist.linear.y = -0.1;
  command_twist.angular.z = 0.3;

  lekiwi_base_controller::UpdateOdometryForSource(
    odometry,
    "command",
    "speed",
    command_twist,
    0.5,
    lekiwi_base_controller::WheelPositionDelta{10.0, 20.0, 30.0},
    kinematics);
  const auto msg = odometry.ToMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", command_twist);

  EXPECT_NEAR(msg.pose.pose.position.x, 0.1, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, -0.05, 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(0.15 / 2.0), 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(0.15 / 2.0), 1e-12);
}

TEST(BaseControllerJointStateTest, BuildsWheelJointStateWithUrdfWheelJointNamesAndVelocities)
{
  const auto msg = lekiwi_base_controller::MakeBaseWheelJointState(
    lekiwi_base_controller::WheelCommand{1.0, -2.0, 3.0},
    {0.25, -0.5, 1.5},
    rclcpp::Time(12, 340000000));

  EXPECT_EQ(msg.header.stamp, rclcpp::Time(12, 340000000));
  EXPECT_EQ(
    msg.name,
    (std::vector<std::string>{"left_wheel_joint", "back_wheel_joint", "right_wheel_joint"}));
  EXPECT_EQ(msg.position, (std::vector<double>{0.25, -0.5, 1.5}));
  EXPECT_EQ(msg.velocity, (std::vector<double>{1.0, -2.0, 3.0}));
  EXPECT_TRUE(msg.effort.empty());
}

TEST(BaseControllerJointStateTest, WheelAnglesIntegratePositionDeltasAndStayWrapped)
{
  const double pi = std::acos(-1.0);
  std::array<double, lekiwi_base_controller::WHEEL_COUNT> angles{};

  lekiwi_base_controller::IntegrateWheelAngles(&angles, std::nullopt);
  EXPECT_EQ(angles, (std::array<double, 3>{0.0, 0.0, 0.0}));

  lekiwi_base_controller::IntegrateWheelAngles(
    &angles, lekiwi_base_controller::WheelPositionDelta{0.1, -0.2, 0.3});
  lekiwi_base_controller::IntegrateWheelAngles(
    &angles, lekiwi_base_controller::WheelPositionDelta{0.1, -0.2, 0.3});
  EXPECT_NEAR(angles[0], 0.2, 1e-12);
  EXPECT_NEAR(angles[1], -0.4, 1e-12);
  EXPECT_NEAR(angles[2], 0.6, 1e-12);

  // 한 바퀴를 넘으면 [-pi, pi]로 감는다
  lekiwi_base_controller::IntegrateWheelAngles(
    &angles, lekiwi_base_controller::WheelPositionDelta{2.0 * pi, 3.0, -4.0});
  EXPECT_NEAR(angles[0], 0.2, 1e-9);
  EXPECT_NEAR(angles[1], -0.4 + 3.0, 1e-9);           // 2.6: 감을 필요 없음
  EXPECT_NEAR(angles[2], 0.6 - 4.0 + 2.0 * pi, 1e-9); // -3.4 → +2.88
  for (const double angle : angles) {
    EXPECT_LE(std::abs(angle), pi);
  }
}

TEST(BaseControllerJointStateTest, EncoderSourceUsesFreshEncoderSpeedsForJointState)
{
  const auto selected = lekiwi_base_controller::SelectJointStateWheelSpeeds(
    lekiwi_base_controller::WheelCommand{1.0, 2.0, 3.0},
    lekiwi_base_controller::WheelCommand{-1.0, -2.0, -3.0},
    "encoder");

  EXPECT_DOUBLE_EQ(selected.left, -1.0);
  EXPECT_DOUBLE_EQ(selected.back, -2.0);
  EXPECT_DOUBLE_EQ(selected.right, -3.0);
}

TEST(BaseControllerJointStateTest, EncoderSourceFallsBackToCommandSpeedsWithoutEncoderSample)
{
  const auto selected = lekiwi_base_controller::SelectJointStateWheelSpeeds(
    lekiwi_base_controller::WheelCommand{1.0, 2.0, 3.0},
    std::nullopt,
    "encoder");

  EXPECT_DOUBLE_EQ(selected.left, 1.0);
  EXPECT_DOUBLE_EQ(selected.back, 2.0);
  EXPECT_DOUBLE_EQ(selected.right, 3.0);
}

TEST(BaseControllerServiceTest, ResetOdometryHelperResetsPoseAndReportsSuccess)
{
  lekiwi_base_controller::Odometry odometry;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 0.5;
  odometry.Update(twist, 1.0);

  const auto response = lekiwi_base_controller::ResetOdometryForService(odometry);
  const auto msg = odometry.ToMessage(rclcpp::Time(0, 0), "odom", "base_footprint", twist);

  EXPECT_TRUE(response.success);
  EXPECT_FALSE(response.message.empty());
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.orientation.w, 1.0);
}

TEST(OdometryTest, AppliesBodyPoseDeltaExactlyOnceInWorldFrame)
{
  lekiwi_base_controller::Odometry odometry;
  odometry.UpdateDelta({0.0, 0.0, M_PI_2});
  odometry.UpdateDelta({1.0, 0.25, 0.0});

  const auto msg = odometry.ToMessage(
    rclcpp::Time(1, 0), "odom", "base_footprint", geometry_msgs::msg::Twist{});
  EXPECT_NEAR(msg.pose.pose.position.x, -0.25, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, 1.0, 1e-12);
}

TEST(BaseControllerOdometrySourceTest, EncoderReadIsThrottledByConfiguredPeriod)
{
  EXPECT_FALSE(lekiwi_base_controller::ShouldRefreshEncoderSample("encoder", 0.0, 0.1));
  EXPECT_FALSE(lekiwi_base_controller::ShouldRefreshEncoderSample("encoder", 0.05, 0.1));
  EXPECT_TRUE(lekiwi_base_controller::ShouldRefreshEncoderSample("encoder", 0.1, 0.1));
  EXPECT_FALSE(lekiwi_base_controller::ShouldRefreshEncoderSample("command", 0.2, 0.1));
}

TEST(BaseControllerOdometrySourceTest, StaleEncoderSampleIsIgnored)
{
  const lekiwi_base_controller::WheelCommand wheel_speeds{1.0, 2.0, 3.0};

  const auto fresh = lekiwi_base_controller::SelectFreshEncoderWheelSpeeds(wheel_speeds, 0.19, 0.2);
  ASSERT_TRUE(fresh.has_value());
  EXPECT_DOUBLE_EQ(fresh->left, wheel_speeds.left);
  EXPECT_DOUBLE_EQ(fresh->back, wheel_speeds.back);
  EXPECT_DOUBLE_EQ(fresh->right, wheel_speeds.right);

  EXPECT_FALSE(lekiwi_base_controller::SelectFreshEncoderWheelSpeeds(wheel_speeds, 0.21, 0.2).has_value());
  EXPECT_FALSE(lekiwi_base_controller::SelectFreshEncoderWheelSpeeds(std::nullopt, 0.0, 0.2).has_value());
}

TEST(BaseControllerEncoderTimingTest, FailedAttemptLeavesLastSuccessfulSampleTimeUnchanged)
{
  std::optional<lekiwi_base_controller::WheelCommand> last_sample;
  std::optional<rclcpp::Time> last_success_time;
  const rclcpp::Time successful_attempt(10, 0);

  EXPECT_TRUE(lekiwi_base_controller::UpdateEncoderFeedbackOnSuccessfulRead(
      lekiwi_base_controller::WheelFeedbackSample{
        lekiwi_base_controller::WheelCommand{1.0, 2.0, 3.0},
        lekiwi_base_controller::WheelPositionDelta{4.0, 5.0, 6.0}},
      successful_attempt,
      &last_sample,
      &last_success_time));
  ASSERT_TRUE(last_success_time.has_value());
  EXPECT_EQ(*last_success_time, successful_attempt);

  EXPECT_FALSE(lekiwi_base_controller::UpdateEncoderFeedbackOnSuccessfulRead(
      std::nullopt,
      rclcpp::Time(20, 0),
      &last_sample,
      &last_success_time));
  ASSERT_TRUE(last_success_time.has_value());
  EXPECT_EQ(*last_success_time, successful_attempt);
  ASSERT_TRUE(last_sample.has_value());
  EXPECT_DOUBLE_EQ(last_sample->left, 1.0);
  EXPECT_DOUBLE_EQ(last_sample->back, 2.0);
  EXPECT_DOUBLE_EQ(last_sample->right, 3.0);
}

TEST(BaseControllerEncoderTimingTest, PostReadTimeDetectsTimeoutCrossedDuringRead)
{
  const std::optional<rclcpp::Time> wait_start{rclcpp::Time(9, 700000000)};
  const std::optional<rclcpp::Time> last_success{rclcpp::Time(9, 800000000)};

  EXPECT_FALSE(lekiwi_base_controller::IsEncoderFeedbackTimedOut(
      wait_start, last_success, rclcpp::Time(10, 0), 0.3));
  EXPECT_TRUE(lekiwi_base_controller::IsEncoderFeedbackTimedOut(
      wait_start, last_success, rclcpp::Time(10, 200000000), 0.3));
  EXPECT_FALSE(lekiwi_base_controller::IsEncoderFeedbackTimedOut(
      wait_start, std::nullopt, rclcpp::Time(10, 0), 0.3));
  EXPECT_TRUE(lekiwi_base_controller::IsEncoderFeedbackTimedOut(
      wait_start, std::nullopt, rclcpp::Time(10, 1), 0.3));
  EXPECT_FALSE(lekiwi_base_controller::IsEncoderFeedbackTimedOut(
      std::nullopt, std::nullopt, rclcpp::Time(10, 200000000), 0.3));
}

TEST(BaseControllerCommandTimingTest, PostReadTimeStopsCommandThatExpiresDuringRead)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.1;
  const rclcpp::Time last_command_time(9, 800000000);

  const auto before_read = lekiwi_base_controller::SelectCommandAtTime(
    command, true, last_command_time, rclcpp::Time(10, 0), 0.3);
  const auto after_read = lekiwi_base_controller::SelectCommandAtTime(
    command, true, last_command_time, rclcpp::Time(10, 200000000), 0.3);

  EXPECT_DOUBLE_EQ(before_read.linear.x, 0.1);
  EXPECT_DOUBLE_EQ(after_read.linear.x, 0.0);
}

TEST(BaseControllerOdometrySourceTest, UnknownSourceUsesCommandTwist)
{
  const lekiwi_base_controller::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto selected = lekiwi_base_controller::SelectOdometryUpdateTwist(
    command, std::nullopt, "wheel_encoder", kinematics);

  EXPECT_DOUBLE_EQ(selected.linear.x, command.linear.x);
  EXPECT_DOUBLE_EQ(selected.linear.y, command.linear.y);
  EXPECT_DOUBLE_EQ(selected.angular.z, command.angular.z);
}

TEST(BaseControllerOdometryTest, TfTimeOffsetOnlyFutureDatesTransformStamp)
{
  const rclcpp::Time stamp(10, 100000000);

  EXPECT_EQ(lekiwi_base_controller::OffsetTransformStamp(stamp, 0.0), stamp);
  EXPECT_EQ(lekiwi_base_controller::OffsetTransformStamp(stamp, 0.05), rclcpp::Time(10, 150000000));
}

TEST(OdometryMessageTest, CommandBasedOdomUsesExplicitNonZeroCovariance)
{
  lekiwi_base_controller::Odometry odometry;
  geometry_msgs::msg::Twist twist;
  const auto msg = odometry.ToMessage(rclcpp::Time(0, 0), "odom", "base_footprint", twist);

  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.child_frame_id, "base_footprint");
  EXPECT_GT(msg.pose.covariance[0], 0.0);
  EXPECT_GT(msg.pose.covariance[7], 0.0);
  EXPECT_GT(msg.pose.covariance[35], 0.0);
  EXPECT_GT(msg.twist.covariance[0], 0.0);
  EXPECT_GT(msg.twist.covariance[7], 0.0);
  EXPECT_GT(msg.twist.covariance[35], 0.0);
  EXPECT_TRUE(std::all_of(
    msg.pose.covariance.begin(), msg.pose.covariance.end(),
    [](const double value) { return std::isfinite(value); }));
  EXPECT_TRUE(std::all_of(
    msg.twist.covariance.begin(), msg.twist.covariance.end(),
    [](const double value) { return std::isfinite(value); }));
}

TEST(BaseControllerParameterValidationTest, AcceptsValidDriverHardwareParameters)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;

  EXPECT_NO_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params));
}

TEST(BaseControllerParameterValidationTest, PublicDefaultsDisableDeprecatedPositionDeadband)
{
  const lekiwi_base_controller::BaseControllerParameters params;

  EXPECT_EQ(params.odom_source, "encoder");
  EXPECT_EQ(params.encoder_position_tick_deadband, 0);
}

TEST(BaseControllerParameterValidationTest, RejectsInvalidEncoderTimingParameters)
{
  lekiwi_base_controller::BaseControllerParameters params;

  params.encoder_read_period = 0.0;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);

  params.encoder_read_period = 0.1;
  params.encoder_sample_timeout = -0.1;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);

  params.encoder_sample_timeout = std::numeric_limits<double>::infinity();
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsDeprecatedPositionDeadbandForPositionFeedback)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.encoder_feedback_source = "position";
  params.encoder_position_tick_deadband = 1;

  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsDeprecatedPositionDeadbandForSpeedFeedback)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.encoder_feedback_source = "speed";
  params.encoder_position_tick_deadband = 4;

  EXPECT_NO_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsPositionFeedbackWindowShorterThanThreeReads)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.encoder_feedback_source = "position";
  params.encoder_read_period = 0.1001;
  params.encoder_sample_timeout = 0.3;

  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsPositionFeedbackWindowAtThreeReadBoundary)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.encoder_feedback_source = "position";
  params.encoder_read_period = 0.1;
  params.encoder_sample_timeout = 0.3;

  EXPECT_NO_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, DoesNotApplyPositionWindowTimingToSpeedFeedback)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.encoder_feedback_source = "speed";
  params.encoder_read_period = 0.1001;
  params.encoder_sample_timeout = 0.3;

  EXPECT_NO_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsInvalidBodyVelocityEnvelope)
{
  lekiwi_base_controller::BaseControllerParameters params;

  params.max_linear_x = 0.0;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);

  params.max_linear_x = 0.18;
  params.max_linear_y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);

  params.max_linear_y = 0.15;
  params.max_linear_speed = -0.18;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);

  params.max_linear_speed = 0.18;
  params.max_angular_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsReferenceGeometryAndVelocityLimit)
{
  lekiwi_base_controller::BaseControllerParameters params;
  EXPECT_DOUBLE_EQ(params.wheel_radius, 0.05);
  EXPECT_DOUBLE_EQ(params.base_radius, 0.13647);
  // 코드 기본값의 바퀴 속도 상한은 드라이버 기본 tick 1500 (tick/s = rad/s × 4096 / 2pi).
  // 실물 설정 파일은 tick 3000이다 (아래 시험)
  EXPECT_NEAR(params.max_wheel_speed * 4096.0 / (2.0 * std::acos(-1.0)), 1500.0, 1e-9);
  // speed 피드백 불감대는 이전 encoder_tick_deadband 50과 같다
  EXPECT_DOUBLE_EQ(params.encoder_speed_deadband, 50.0 * 2.0 * std::acos(-1.0) / 4096.0);
  EXPECT_NO_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsIndividuallyInfeasibleAxis)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.max_linear_speed = 0.56;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
  params.max_linear_speed = 0.26;
  params.max_angular_z = 5.0;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsRealConfigLimitsAndScalesMixedCommands)
{
  // 실물 설정 (2026-10-22): 바퀴 tick 3000, 몸체는 축 하나만 최대여도 바퀴 상한 안에 드는 0.23 m/s · 1.53 rad/s.
  // 사용자가 바란 0.3 m/s · 90°/s는 바퀴 상한을 넘어 검사에 걸린다
  lekiwi_base_controller::BaseControllerParameters params;
  params.kinematics_frame_x = 0.007587843843;
  params.kinematics_frame_y = -0.010776931094;
  params.max_linear_x = 0.23;
  params.max_linear_y = 0.23;
  params.max_linear_speed = 0.23;
  params.max_angular_z = 1.53;
  params.max_wheel_speed = 3000.0 / 651.8986469044033;
  EXPECT_NO_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params));
  params.max_linear_speed = 0.3;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
  params.max_linear_speed = 0.23;
  params.max_angular_z = 1.5707963;
  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
  params.max_angular_z = 1.53;

  // 직선 + 회전을 섞어 바퀴 상한을 넘으면 세 바퀴를 같은 비율로 줄인다: 가장 빠른 바퀴가 상한, 방향은 그대로
  const lekiwi_base_controller::OmniKinematics kinematics(
    params.wheel_radius, params.base_radius, params.max_wheel_speed,
    params.kinematics_frame_x, params.kinematics_frame_y, params.kinematics_frame_yaw);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.23;
  command.angular.z = 1.53;
  const auto limited = lekiwi_base_controller::LimitBodyTwist(
    command, params.max_linear_x, params.max_linear_y, params.max_linear_speed,
    params.max_angular_z, kinematics);
  EXPECT_LT(limited.linear.x, 0.23);
  EXPECT_NEAR(limited.linear.y, 0.0, 1e-12);
  EXPECT_NEAR(limited.angular.z / limited.linear.x, 1.53 / 0.23, 1e-9);
  const auto wheels = kinematics.ToWheelSpeeds(
    limited.linear.x, limited.linear.y, limited.angular.z);
  EXPECT_NEAR(
    std::max({std::abs(wheels.left), std::abs(wheels.back), std::abs(wheels.right)}),
    params.max_wheel_speed, 1e-9);
}

TEST(BaseControllerParameterValidationTest, RejectsWheelIdsOutsideServoRange)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;

  params.wheel_ids = {7, 0, 9};
  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);

  params.wheel_ids = {7, 254, 9};
  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsDuplicateWheelIds)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;
  params.wheel_ids = {7, 7, 9};

  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsZeroWheelDirection)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;
  params.wheel_directions = {1.0, 0.0, -1.0};

  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsPositionDeadbandOutsideHalfRevolution)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;

  params.encoder_position_tick_deadband = -1;
  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);

  params.encoder_position_tick_deadband = 2048;
  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);

  params.encoder_position_tick_deadband = 2047;
  EXPECT_NO_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsUnknownEncoderFeedbackSource)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.encoder_feedback_source = "velocity";

  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsFractionalPositionTicksPerRevolution)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;
  params.encoder_position_ticks_per_revolution = 4096.5;

  EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsNonPositiveOrNonFiniteEncoderOdomScale)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;

  for (const double invalid_scale : {
      0.0,
      -1.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    params.encoder_odom_scale = invalid_scale;
    EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params), std::invalid_argument);
  }
}

TEST(BaseControllerParameterValidationTest, RejectsInvalidDriverTimingAndDeadband)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;

  for (const double invalid : {0.0, -0.5, std::numeric_limits<double>::infinity()}) {
    auto bad = params;
    bad.driver_states_timeout = invalid;
    EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(bad), std::invalid_argument);
    bad = params;
    bad.torque_service_timeout = invalid;
    EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(bad), std::invalid_argument);
  }
  for (const double invalid : {-0.01, std::numeric_limits<double>::quiet_NaN()}) {
    auto bad = params;
    bad.encoder_speed_deadband = invalid;
    EXPECT_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(bad), std::invalid_argument);
  }
  params.encoder_speed_deadband = 0.0;
  EXPECT_NO_THROW(lekiwi_base_controller::ValidateDriverMotorParameters(params));
}

// 토크는 lekiwi/motor/power 서비스로만 켠다
TEST(BaseControllerParameterValidationTest, RejectsTorqueEnableAtStartup)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.is_torque_enable_requested = true;

  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
  params.is_torque_enable_requested = false;
  EXPECT_NO_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsEmptyTopicName)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.wheel_torque_service = "";

  EXPECT_THROW(lekiwi_base_controller::ValidateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, DefaultsUseInterfaceContractNames)
{
  const lekiwi_base_controller::BaseControllerParameters params;

  EXPECT_EQ(params.cmd_vel_topic, "cmd_vel");
  EXPECT_EQ(params.odom_topic, "odom");
  EXPECT_EQ(params.wheel_joint_states_topic, "lekiwi/motor/wheel_joint_states");
  EXPECT_EQ(params.motor_ready_topic, "lekiwi/motor/ready");
  EXPECT_EQ(params.motor_power_service, "lekiwi/motor/power");
  EXPECT_EQ(params.reset_odometry_service, "lekiwi/motor/reset_odometry");
  EXPECT_EQ(params.states_topic, "lekiwi/motor/states");
  EXPECT_EQ(params.wheel_command_topic, "lekiwi/motor/wheel_command");
  EXPECT_EQ(params.wheel_torque_service, "lekiwi/motor/wheel_torque");
  // 노드 기본값은 안전 쪽 (mock, 쓰기 꺼짐)
  EXPECT_EQ(params.motor_backend, "mock");
  EXPECT_FALSE(params.is_motor_write_enabled);
  EXPECT_FALSE(params.is_torque_enable_requested);
}

TEST(BaseControllerBackendTest, LegacyFeetechBackendNameNoLongerSelectsHardware)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.is_motor_write_enabled = true;

  EXPECT_EQ(
    lekiwi_base_controller::BaseMotorBackendKind::MockLogging,
    lekiwi_base_controller::SelectBaseMotorBackend(params));
}

TEST(BaseControllerConfigContractTest, RealConfigSelectsDriverWithTorqueOff)
{
  const auto yaml = ReadSourceFile("config/lekiwi_base_controller.yaml");

  EXPECT_EQ(yaml.find("lekiwi_base_controller_node:\n  ros__parameters:"), 0u);
  EXPECT_NE(yaml.find("motor_backend: driver"), std::string::npos);
  EXPECT_NE(yaml.find("enable_motor_write: true"), std::string::npos);
  EXPECT_NE(yaml.find("torque_enable: false"), std::string::npos);
  EXPECT_NE(yaml.find("odom_source: encoder"), std::string::npos);
  EXPECT_NE(yaml.find("encoder_feedback_source: position"), std::string::npos);
  EXPECT_NE(yaml.find("wheel_ids: [8, 9, 7]"), std::string::npos);
  EXPECT_NE(yaml.find("cmd_vel_timeout: 0.3"), std::string::npos);
  EXPECT_NE(yaml.find("encoder_sample_timeout: 0.5"), std::string::npos);
}
