#include "lekiwi_base_controller/lekiwi_base_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "lekiwi_base_controller/driver_motor_interface.hpp"

namespace lekiwi_base_controller
{
namespace
{
constexpr auto CONTROL_PERIOD = std::chrono::milliseconds(20);   // 50 Hz

bool IsFiniteTwist(const geometry_msgs::msg::Twist & twist)
{
  return
    std::isfinite(twist.linear.x) &&
    std::isfinite(twist.linear.y) &&
    std::isfinite(twist.linear.z) &&
    std::isfinite(twist.angular.x) &&
    std::isfinite(twist.angular.y) &&
    std::isfinite(twist.angular.z);
}

bool IsSameTwist(
  const geometry_msgs::msg::Twist & lhs,
  const geometry_msgs::msg::Twist & rhs)
{
  return
    lhs.linear.x == rhs.linear.x &&
    lhs.linear.y == rhs.linear.y &&
    lhs.linear.z == rhs.linear.z &&
    lhs.angular.x == rhs.angular.x &&
    lhs.angular.y == rhs.angular.y &&
    lhs.angular.z == rhs.angular.z;
}

const char * ToString(const EncoderReadStatus status)
{
  switch (status) {
    case EncoderReadStatus::Sample:
      return "sample";
    case EncoderReadStatus::NoSample:
      return "no_sample";
    case EncoderReadStatus::Exception:
      return "exception";
    case EncoderReadStatus::NotAttempted:
    default:
      return "not_attempted";
  }
}
}  // namespace

LekiwiBaseControllerNode::LekiwiBaseControllerNode(const rclcpp::NodeOptions & options)
: Node("lekiwi_base_controller_node", options),
  params_(LoadParameters()),
  kinematics_(
    params_.wheel_radius, params_.base_radius, params_.max_wheel_speed,
    params_.kinematics_frame_x, params_.kinematics_frame_y, params_.kinematics_frame_yaw),
  last_cmd_time_(now()),
  last_update_time_(now())
{
  ValidateBaseControllerParameters(params_);
  WarnAboutBackendSelection();

  CreateMotorInterface();
  StartMotorInterface();
  const rclcpp::Time initial_time = now();
  motor_safety_ = std::make_unique<MotorSafetyCoordinator>(
    *motor_interface_,
    IsRealMotorHardware(),
    IsRealMotorHardware() || params_.odom_source == "encoder",
    params_.encoder_read_period,
    params_.encoder_sample_timeout,
    initial_time,
    [this]() {return now();});
  CreateRosInterfaces();

  if (IsRealMotorHardware()) {
    // Ctrl+C로 컨텍스트가 닫히기 전에 속도 0을 보낸다 (닫힌 뒤에는 발행할 수 없다)
    pre_shutdown_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {StopMotorInterfaceForShutdown(motor_interface_.get());});
  }
  RCLCPP_INFO(
    get_logger(), "motor backend: %s; odom source: %s (%s feedback); publishing %s, %s",
    IsRealMotorHardware() ? "driver (real wheels)" : "mock (logging only)",
    params_.odom_source.c_str(), params_.encoder_feedback_source.c_str(),
    odom_pub_->get_topic_name(), motor_ready_pub_->get_topic_name());
}

LekiwiBaseControllerNode::~LekiwiBaseControllerNode()
{
  if (pre_shutdown_handle_) {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(*pre_shutdown_handle_);
  }
  if (control_timer_) {
    control_timer_->cancel();
  }
  StopMotorInterfaceForShutdown(motor_interface_.get());
}

/*********************************************************
  파라미터
 *********************************************************/

BaseControllerParameters LekiwiBaseControllerNode::LoadParameters()
{
  BaseControllerParameters params;
  // 기구학
  params.wheel_radius = declare_parameter<double>("wheel_radius", params.wheel_radius);
  params.base_radius = declare_parameter<double>("base_radius", params.base_radius);
  params.kinematics_frame_x =
    declare_parameter<double>("kinematics_frame_x", params.kinematics_frame_x);
  params.kinematics_frame_y =
    declare_parameter<double>("kinematics_frame_y", params.kinematics_frame_y);
  params.kinematics_frame_yaw =
    declare_parameter<double>("kinematics_frame_yaw", params.kinematics_frame_yaw);
  // 속도 제한
  params.max_linear_x = declare_parameter<double>("max_linear_x", params.max_linear_x);
  params.max_linear_y = declare_parameter<double>("max_linear_y", params.max_linear_y);
  params.max_linear_speed =
    declare_parameter<double>("max_linear_speed", params.max_linear_speed);
  params.max_angular_z = declare_parameter<double>("max_angular_z", params.max_angular_z);
  params.max_wheel_speed = declare_parameter<double>("max_wheel_speed", params.max_wheel_speed);
  // 오도메트리
  params.odom_frame = declare_parameter<std::string>("odom_frame", params.odom_frame);
  params.base_frame = declare_parameter<std::string>("base_frame", params.base_frame);
  params.cmd_vel_timeout = declare_parameter<double>("cmd_vel_timeout", params.cmd_vel_timeout);
  params.tf_time_offset = declare_parameter<double>("tf_time_offset", params.tf_time_offset);
  params.odom_source = declare_parameter<std::string>("odom_source", params.odom_source);
  params.encoder_read_period =
    declare_parameter<double>("encoder_read_period", params.encoder_read_period);
  params.encoder_sample_timeout =
    declare_parameter<double>("encoder_sample_timeout", params.encoder_sample_timeout);
  // 모터 백엔드
  params.motor_backend = declare_parameter<std::string>("motor_backend", params.motor_backend);
  params.is_motor_write_enabled =
    declare_parameter<bool>("enable_motor_write", params.is_motor_write_enabled);
  params.wheel_ids = declare_parameter<std::vector<int64_t>>("wheel_ids", params.wheel_ids);
  params.wheel_directions =
    declare_parameter<std::vector<double>>("wheel_directions", params.wheel_directions);
  params.encoder_feedback_source =
    declare_parameter<std::string>("encoder_feedback_source", params.encoder_feedback_source);
  params.encoder_position_ticks_per_revolution =
    declare_parameter<double>(
    "encoder_position_ticks_per_revolution",
    params.encoder_position_ticks_per_revolution);
  params.encoder_odom_scale =
    declare_parameter<double>("encoder_odom_scale", params.encoder_odom_scale);
  params.encoder_position_tick_deadband =
    declare_parameter<int64_t>(
    "encoder_position_tick_deadband",
    params.encoder_position_tick_deadband);
  params.encoder_speed_deadband =
    declare_parameter<double>("encoder_speed_deadband", params.encoder_speed_deadband);
  params.is_torque_enable_requested =
    declare_parameter<bool>("torque_enable", params.is_torque_enable_requested);
  params.is_encoder_read_logged =
    declare_parameter<bool>("log_encoder_reads", params.is_encoder_read_logged);
  params.driver_states_timeout =
    declare_parameter<double>("driver_states_timeout", params.driver_states_timeout);
  params.torque_service_timeout =
    declare_parameter<double>("torque_service_timeout", params.torque_service_timeout);
  // 토픽 · 서비스
  params.cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", params.cmd_vel_topic);
  params.odom_topic = declare_parameter<std::string>("odom_topic", params.odom_topic);
  params.wheel_joint_states_topic = declare_parameter<std::string>(
    "wheel_joint_states_topic", params.wheel_joint_states_topic);
  params.motor_ready_topic =
    declare_parameter<std::string>("motor_ready_topic", params.motor_ready_topic);
  params.motor_power_service =
    declare_parameter<std::string>("motor_power_service", params.motor_power_service);
  params.reset_odometry_service =
    declare_parameter<std::string>("reset_odometry_service", params.reset_odometry_service);
  params.states_topic = declare_parameter<std::string>("states_topic", params.states_topic);
  params.wheel_command_topic =
    declare_parameter<std::string>("wheel_command_topic", params.wheel_command_topic);
  params.wheel_torque_service =
    declare_parameter<std::string>("wheel_torque_service", params.wheel_torque_service);
  return params;
}

void LekiwiBaseControllerNode::WarnAboutBackendSelection() const
{
  if (params_.motor_backend == "driver") {
    if (!params_.is_motor_write_enabled) {
      RCLCPP_WARN(
        get_logger(),
        "motor_backend is 'driver' but enable_motor_write is false; wheel commands will not be "
        "sent to the motor driver; using mock/logging backend");
    }
  } else if (params_.motor_backend != "mock") {
    RCLCPP_WARN(
      get_logger(),
      "unknown motor_backend '%s'; hardware interface will not be selected; using mock/logging "
      "backend",
      params_.motor_backend.c_str());
  }
  if (params_.odom_source != "command" && params_.odom_source != "encoder") {
    RCLCPP_WARN(
      get_logger(),
      "unknown odom_source '%s'; falling back to command odometry",
      params_.odom_source.c_str());
  }
}

/*********************************************************
  시작
 *********************************************************/

void LekiwiBaseControllerNode::CreateMotorInterface()
{
  const auto backend = SelectBaseMotorBackend(params_);
  if (backend == BaseMotorBackendKind::DriverHardware) {
    ValidateDriverMotorParameters(params_);
    motor_interface_ = std::make_unique<DriverMotorInterface>(
      *this, MakeDriverMotorConfig(params_));
    return;
  }
  motor_interface_ = std::make_unique<LoggingMotorInterface>(get_logger(), params_.max_wheel_speed);
}

void LekiwiBaseControllerNode::StartMotorInterface()
{
  if (motor_interface_) {
    motor_interface_->Connect();
  }
}

void LekiwiBaseControllerNode::CreateRosInterfaces()
{
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    params_.odom_topic, rclcpp::SystemDefaultsQoS());
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    params_.wheel_joint_states_topic, rclcpp::SystemDefaultsQoS());
  auto qos = MakeMotorReadyQos();
  motor_ready_pub_ = create_publisher<std_msgs::msg::Bool>(params_.motor_ready_topic, qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rclcpp::SubscriptionOptions cmd_vel_options;
  cmd_vel_options.callback_group = control_group_;
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    params_.cmd_vel_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {CmdVelCallback(msg);},
    cmd_vel_options);
  reset_odometry_srv_ = create_service<std_srvs::srv::Trigger>(
    params_.reset_odometry_service,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      ResetOdometryCallback(request, response);
    },
    rclcpp::ServicesQoS(), control_group_);
  motor_power_srv_ = create_service<std_srvs::srv::SetBool>(
    params_.motor_power_service,
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      MotorPowerCallback(request, response);
    },
    rclcpp::ServicesQoS(), control_group_);
  PublishMotorReady();
  control_timer_ = create_wall_timer(
    CONTROL_PERIOD, [this]() {ControlTimerCallback();}, control_group_);
}

/*********************************************************
  콜백
 *********************************************************/

void LekiwiBaseControllerNode::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto limited = LimitBodyTwist(
    *msg,
    params_.max_linear_x,
    params_.max_linear_y,
    params_.max_linear_speed,
    params_.max_angular_z,
    kinematics_);
  if (!IsFiniteTwist(*msg)) {
    RCLCPP_WARN(get_logger(), "received non-finite cmd_vel; commanding zero velocity");
  } else if (!IsSameTwist(*msg, limited)) {
    RCLCPP_WARN(get_logger(), "cmd_vel exceeded body or wheel velocity limits and was limited");
  }
  current_cmd_ = limited;
  last_cmd_time_ = now();
  has_cmd_ = true;
}

void LekiwiBaseControllerNode::ControlTimerCallback()
{
  const rclcpp::Time current_time = now();
  const double dt = std::max(0.0, (current_time - last_update_time_).seconds());
  last_update_time_ = current_time;

  // (1) 엔코더 신선도 · 연결 확인 (필요하면 속도 0 → 전원 끔 → 고장 고정)
  const auto safety_update = motor_safety_->BeforeCommand(current_time);
  const rclcpp::Time safety_time = safety_update.safety_time;
  if (safety_update.is_readiness_changed) {
    PublishMotorReady();
  }
  if (safety_update.is_encoder_timeout) {
    const char * stage =
      safety_update.encoder_timeout_stage == EncoderTimeoutStage::BeforeRead ?
      "before_read" : "after_read";
    RCLCPP_ERROR(
      get_logger(),
      "encoder feedback timed out; motor power disabled and fault latched "
      "(stage=%s sample_age=%.3fs read_status=%s read_duration=%.3fs failed_reads=%u)",
      stage, safety_update.encoder_sample_age_seconds,
      ToString(safety_update.encoder_read_status),
      safety_update.encoder_read_duration_seconds, safety_update.consecutive_failed_reads);
  }

  // (2) cmd_vel_timeout이 지난 명령은 0
  const geometry_msgs::msg::Twist command = SelectCommandAtTime(
    current_cmd_, has_cmd_, last_cmd_time_, safety_time, params_.cmd_vel_timeout);
  const WheelCommand command_wheel_speeds =
    kinematics_.ToWheelSpeeds(command.linear.x, command.linear.y, command.angular.z);

  // (3) 준비 상태가 아니면 속도 0만 나간다
  motor_safety_->SendWheelCommand(command_wheel_speeds);
  const auto encoder_wheel_speeds = motor_safety_->GetFreshEncoderWheelSpeeds(safety_time);
  const geometry_msgs::msg::Twist odom_twist =
    SelectOdometryUpdateTwist(command, encoder_wheel_speeds, params_.odom_source, kinematics_);

  // (4) odom · 바퀴 관절 · TF
  UpdateOdometryForSource(
    odometry_,
    params_.odom_source,
    params_.encoder_feedback_source,
    odom_twist,
    dt,
    safety_update.wheel_position_delta,
    kinematics_);
  IntegrateWheelAngles(&wheel_angles_, safety_update.wheel_position_delta);
  odom_pub_->publish(
    odometry_.ToMessage(current_time, params_.odom_frame, params_.base_frame, odom_twist));
  joint_state_pub_->publish(
    MakeBaseWheelJointState(
      SelectJointStateWheelSpeeds(command_wheel_speeds, encoder_wheel_speeds, params_.odom_source),
      wheel_angles_,
      current_time));
  tf_broadcaster_->sendTransform(
    odometry_.ToTransform(
      OffsetTransformStamp(current_time, params_.tf_time_offset),
      params_.odom_frame,
      params_.base_frame));
}

void LekiwiBaseControllerNode::ResetOdometryCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  *response = ResetOdometryForService(odometry_);
}

void LekiwiBaseControllerNode::MotorPowerCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // 드라이버 토크 서비스를 동기 호출한다 (control_group_ 밖의 클라이언트 그룹이 응답을 받는다)
  *response = motor_safety_->SetMotorPower(request->data, now());
  PublishMotorReady();
  if (response->success) {
    RCLCPP_INFO(get_logger(), "motor power request %s: %s",
      request->data ? "on" : "off", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "motor power request %s: %s",
      request->data ? "on" : "off", response->message.c_str());
  }
}

void LekiwiBaseControllerNode::PublishMotorReady()
{
  if (!motor_ready_pub_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = motor_safety_->IsMotorReady();
  motor_ready_pub_->publish(message);
}

bool LekiwiBaseControllerNode::IsRealMotorHardware() const
{
  return SelectBaseMotorBackend(params_) == BaseMotorBackendKind::DriverHardware;
}

}  // namespace lekiwi_base_controller
