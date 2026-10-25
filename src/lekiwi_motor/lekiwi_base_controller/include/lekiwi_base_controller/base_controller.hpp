#ifndef LEKIWI_BASE_CONTROLLER__BASE_CONTROLLER_HPP_
#define LEKIWI_BASE_CONTROLLER__BASE_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lekiwi_base_controller/motor_readiness.hpp"
#include "lekiwi_base_controller/odometry.hpp"
#include "lekiwi_base_controller/omni_kinematics.hpp"
#include "lekiwi_base_controller/wheel_feedback_estimator.hpp"

// 베이스 제어 공통 부분: 파라미터, 명령 제한 · 오도메트리 선택 함수, 모터 인터페이스 기반 클래스
namespace lekiwi_base_controller
{

struct BaseControllerParameters
{
  // 기구학 (lekiwi_description/config/base_frame.yaml과 같은 값)
  double wheel_radius{0.05};
  double base_radius{0.13647};
  double kinematics_frame_x{0.007587843843};
  double kinematics_frame_y{-0.010776931094};
  double kinematics_frame_yaw{2.094379865161};
  // 속도 제한
  double max_linear_x{0.11};
  double max_linear_y{0.1};
  double max_linear_speed{0.11};
  double max_angular_z{0.75};
  double max_wheel_speed{2.3009711818284617};
  // 오도메트리
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  double cmd_vel_timeout{0.3};
  double tf_time_offset{0.0};
  std::string odom_source{"encoder"};
  double encoder_read_period{0.1};
  double encoder_sample_timeout{0.5};
  // 모터 백엔드
  std::string motor_backend{"mock"};
  bool is_motor_write_enabled{false};          // enable_motor_write
  // 모델 관절 left/back/right는 실물 뒤/오른쪽/왼쪽 바퀴 자리다
  std::vector<int64_t> wheel_ids{8, 9, 7};
  std::vector<double> wheel_directions{1.0, 1.0, 1.0};
  std::string encoder_feedback_source{"position"};
  double encoder_position_ticks_per_revolution{4096.0};
  double encoder_odom_scale{1.0};
  int64_t encoder_position_tick_deadband{0};
  double encoder_speed_deadband{50.0 / 651.8986469044033};  // [rad/s] tick 50과 같다
  bool is_torque_enable_requested{false};      // torque_enable (항상 false여야 한다)
  bool is_encoder_read_logged{false};          // log_encoder_reads
  double driver_states_timeout{0.5};
  double torque_service_timeout{1.0};
  // 토픽 · 서비스 이름 (docs/interfaces.md)
  std::string cmd_vel_topic{"cmd_vel"};
  std::string odom_topic{"odom"};
  std::string wheel_joint_states_topic{"lekiwi/motor/wheel_joint_states"};
  std::string motor_ready_topic{"lekiwi/motor/ready"};
  std::string motor_power_service{"lekiwi/motor/power"};
  std::string reset_odometry_service{"lekiwi/motor/reset_odometry"};
  std::string states_topic{"lekiwi/motor/states"};
  std::string wheel_command_topic{"lekiwi/motor/wheel_command"};
  std::string wheel_torque_service{"lekiwi/motor/wheel_torque"};
};

enum class BaseMotorBackendKind
{
  MockLogging,     // 로그만 남김. 실물 준비 상태가 되지 않는다
  DriverHardware,  // lekiwi_motor_driver_node를 통해 실제 바퀴를 움직인다
};

class BaseMotorInterface;

BaseMotorBackendKind SelectBaseMotorBackend(const BaseControllerParameters & params);
void ValidateBaseControllerParameters(const BaseControllerParameters & params);
void ValidateDriverMotorParameters(const BaseControllerParameters & params);
std::array<uint8_t, WHEEL_COUNT> WheelIdsFromParameter(const std::vector<int64_t> & wheel_ids);
std::array<double, WHEEL_COUNT> WheelDirectionsFromParameter(
  const std::vector<double> & wheel_directions);
geometry_msgs::msg::Twist LimitBodyTwist(
  const geometry_msgs::msg::Twist & command,
  double max_linear_x,
  double max_linear_y,
  double max_linear_speed,
  double max_angular_z,
  const OmniKinematics & kinematics);
geometry_msgs::msg::Twist SelectOdometryTwist(
  const geometry_msgs::msg::Twist & command,
  bool has_command,
  double command_age,
  double command_timeout);
geometry_msgs::msg::Twist SelectCommandAtTime(
  const geometry_msgs::msg::Twist & command,
  bool has_command,
  const rclcpp::Time & last_command_time,
  const rclcpp::Time & current_time,
  double command_timeout);
geometry_msgs::msg::Twist SelectOdometryUpdateTwist(
  const geometry_msgs::msg::Twist & command,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source,
  const OmniKinematics & kinematics);
void UpdateOdometryForSource(
  Odometry & odometry,
  const std::string & odom_source,
  const std::string & encoder_feedback_source,
  const geometry_msgs::msg::Twist & selected_twist,
  double control_dt,
  const std::optional<WheelPositionDelta> & wheel_position_delta,
  const OmniKinematics & kinematics);
bool ShouldRefreshEncoderSample(
  const std::string & odom_source,
  double encoder_sample_age,
  double encoder_read_period);
std::optional<WheelCommand> SelectFreshEncoderWheelSpeeds(
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  double encoder_sample_age,
  double encoder_sample_timeout);
bool UpdateEncoderFeedbackOnSuccessfulRead(
  const std::optional<WheelFeedbackSample> & read_result,
  const rclcpp::Time & success_time,
  std::optional<WheelCommand> * last_sample,
  std::optional<rclcpp::Time> * last_success_time);
bool IsEncoderFeedbackTimedOut(
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  double encoder_sample_timeout);
bool IsMotorPowerEnableAllowed(
  const MotorReadinessLatch & motor_readiness,
  bool is_recovery_disarmed);
// 엔코더 마감 시간이 지났으면 속도 0 → 전원 끔 → 고장 고정 순서로 처리하고 true
bool StopMotorOnEncoderTimeout(
  BaseMotorInterface & motor_interface,
  MotorReadinessLatch & motor_readiness,
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  double encoder_sample_timeout);
WheelCommand SelectJointStateWheelSpeeds(
  const WheelCommand & command_wheel_speeds,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source);
// 위치 증분을 바퀴 각도에 더한다 (모델 관절 기준, [-pi, pi]로 감음)
void IntegrateWheelAngles(
  std::array<double, WHEEL_COUNT> * wheel_angles,
  const std::optional<WheelPositionDelta> & wheel_position_delta);
sensor_msgs::msg::JointState MakeBaseWheelJointState(
  const WheelCommand & wheel_speeds,
  const std::array<double, WHEEL_COUNT> & wheel_angles,
  const rclcpp::Time & stamp);
std_srvs::srv::Trigger::Response ResetOdometryForService(Odometry & odometry);
std_srvs::srv::SetBool::Response SetMotorPowerForService(
  BaseMotorInterface & motor_interface,
  bool is_enabled);
rclcpp::Time OffsetTransformStamp(const rclcpp::Time & stamp, double offset_seconds);

// 바퀴 모터 백엔드 공통 계약.
//   WriteWheelSpeeds: 전원이 꺼져 있으면 쓰지 않고, 켜져 있으면 ±max_wheel_speed로 자른 뒤 쓴다
class BaseMotorInterface
{
public:
  explicit BaseMotorInterface(double max_wheel_speed);
  virtual ~BaseMotorInterface() = default;
  virtual void Connect() = 0;
  virtual void Shutdown() noexcept;
  virtual bool IsConnected() const;
  virtual std::optional<WheelFeedbackSample> ReadWheelFeedback();
  virtual bool SetMotorPower(bool is_enabled);
  bool IsMotorPowerEnabled() const;
  void WriteWheelSpeeds(const WheelCommand & command);
  void Stop();

protected:
  virtual void WriteClampedWheelSpeeds(const WheelCommand & command) = 0;
  void SetMotorPowerState(bool is_enabled);

private:
  double max_wheel_speed_;
  bool is_motor_power_enabled_{true};
};

void StopMotorInterfaceForShutdown(BaseMotorInterface * motor_interface) noexcept;

// mock 백엔드: 명령을 debug 로그로만 남긴다
class LoggingMotorInterface : public BaseMotorInterface
{
public:
  LoggingMotorInterface(rclcpp::Logger logger, double max_wheel_speed);
  void Connect() override;
  bool IsConnected() const override;

private:
  void WriteClampedWheelSpeeds(const WheelCommand & command) override;

  rclcpp::Logger logger_;
};

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__BASE_CONTROLLER_HPP_
