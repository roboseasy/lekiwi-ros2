#include "lekiwi_base_controller/base_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lekiwi_base_controller
{
namespace
{
constexpr int64_t MAX_SERVO_ID = 253;
constexpr double TWO_PI = 6.28318530717958647692;

void ValidateWheelIdsParameter(const std::vector<int64_t> & wheel_ids)
{
  if (wheel_ids.size() != WHEEL_COUNT) {
    throw std::invalid_argument("wheel_ids must contain exactly 3 values");
  }
  for (std::size_t i = 0; i < wheel_ids.size(); ++i) {
    if (wheel_ids[i] < 1 || wheel_ids[i] > MAX_SERVO_ID) {
      throw std::invalid_argument("wheel_ids must be in the range [1, 253]");
    }
    for (std::size_t j = i + 1; j < wheel_ids.size(); ++j) {
      if (wheel_ids[i] == wheel_ids[j]) {
        throw std::invalid_argument("wheel_ids must not contain duplicate values");
      }
    }
  }
}

void ValidateWheelDirectionsParameter(const std::vector<double> & wheel_directions)
{
  if (wheel_directions.size() != WHEEL_COUNT) {
    throw std::invalid_argument("wheel_directions must contain exactly 3 values");
  }
  for (const double wheel_direction : wheel_directions) {
    if (!std::isfinite(wheel_direction)) {
      throw std::invalid_argument("wheel_directions must be finite");
    }
    if (wheel_direction == 0.0) {
      throw std::invalid_argument("wheel_directions must not contain zero");
    }
  }
}

int IntFromParameter(const int64_t value, const char * parameter_name)
{
  if (value < static_cast<int64_t>(std::numeric_limits<int>::min()) ||
    value > static_cast<int64_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument(std::string(parameter_name) + " must fit in int");
  }
  return static_cast<int>(value);
}

void ValidatePositiveFinite(const double value, const char * parameter_name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(parameter_name) + " must be positive and finite");
  }
}

geometry_msgs::msg::Twist ToRosTwist(const BodyTwist & twist)
{
  geometry_msgs::msg::Twist ros_twist;
  ros_twist.linear.x = twist.linear_x;
  ros_twist.linear.y = twist.linear_y;
  ros_twist.angular.z = twist.angular_z;
  return ros_twist;
}

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
}  // namespace

/*********************************************************
  모터 인터페이스
 *********************************************************/

BaseMotorInterface::BaseMotorInterface(double max_wheel_speed)
: max_wheel_speed_(max_wheel_speed)
{
  if (!std::isfinite(max_wheel_speed_) || max_wheel_speed_ <= 0.0) {
    throw std::invalid_argument("max_wheel_speed must be positive and finite");
  }
}

void BaseMotorInterface::WriteWheelSpeeds(const WheelCommand & command)
{
  if (!is_motor_power_enabled_) {
    return;
  }

  const WheelCommand clamped{
    std::clamp(command.left, -max_wheel_speed_, max_wheel_speed_),
    std::clamp(command.back, -max_wheel_speed_, max_wheel_speed_),
    std::clamp(command.right, -max_wheel_speed_, max_wheel_speed_)};
  WriteClampedWheelSpeeds(clamped);
}

void BaseMotorInterface::Stop()
{
  WriteWheelSpeeds(WheelCommand{});
}

void BaseMotorInterface::Shutdown() noexcept
{
  try {
    Stop();
  } catch (...) {
  }
}

std::optional<WheelFeedbackSample> BaseMotorInterface::ReadWheelFeedback()
{
  return std::nullopt;
}

bool BaseMotorInterface::IsConnected() const
{
  return false;
}

bool BaseMotorInterface::SetMotorPower(const bool is_enabled)
{
  if (!is_enabled) {
    Stop();
  }
  is_motor_power_enabled_ = is_enabled;
  return true;
}

bool BaseMotorInterface::IsMotorPowerEnabled() const
{
  return is_motor_power_enabled_;
}

void BaseMotorInterface::SetMotorPowerState(const bool is_enabled)
{
  is_motor_power_enabled_ = is_enabled;
}

void StopMotorInterfaceForShutdown(BaseMotorInterface * motor_interface) noexcept
{
  if (!motor_interface) {
    return;
  }

  motor_interface->Shutdown();
}

LoggingMotorInterface::LoggingMotorInterface(rclcpp::Logger logger, double max_wheel_speed)
: BaseMotorInterface(max_wheel_speed),
  logger_(logger)
{
}

void LoggingMotorInterface::Connect()
{
}

bool LoggingMotorInterface::IsConnected() const
{
  return false;
}

void LoggingMotorInterface::WriteClampedWheelSpeeds(const WheelCommand & command)
{
  RCLCPP_DEBUG(
    logger_, "wheel command left=%.3f back=%.3f right=%.3f",
    command.left, command.back, command.right);
}

/*********************************************************
  파라미터 검증
 *********************************************************/

BaseMotorBackendKind SelectBaseMotorBackend(const BaseControllerParameters & params)
{
  if (params.motor_backend == "driver" && params.is_motor_write_enabled) {
    return BaseMotorBackendKind::DriverHardware;
  }
  return BaseMotorBackendKind::MockLogging;
}

void ValidateBaseControllerParameters(const BaseControllerParameters & params)
{
  // 토크는 lekiwi/motor/power 서비스로만 켠다
  if (params.is_torque_enable_requested) {
    throw std::invalid_argument(
      "torque_enable must be false; enable wheel torque through the motor power service");
  }
  for (const auto & [name, value] : std::array<std::pair<const char *, const std::string *>, 11>{{
      {"odom_frame", &params.odom_frame},
      {"base_frame", &params.base_frame},
      {"cmd_vel_topic", &params.cmd_vel_topic},
      {"odom_topic", &params.odom_topic},
      {"wheel_joint_states_topic", &params.wheel_joint_states_topic},
      {"motor_ready_topic", &params.motor_ready_topic},
      {"motor_power_service", &params.motor_power_service},
      {"reset_odometry_service", &params.reset_odometry_service},
      {"states_topic", &params.states_topic},
      {"wheel_command_topic", &params.wheel_command_topic},
      {"wheel_torque_service", &params.wheel_torque_service}}})
  {
    if (value->empty()) {
      throw std::invalid_argument(std::string(name) + " must not be empty");
    }
  }

  for (const double value : {
      params.kinematics_frame_x, params.kinematics_frame_y, params.kinematics_frame_yaw})
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("kinematics frame transform must be finite");
    }
  }
  const std::array<std::pair<double, const char *>, 4> body_limits{{
      {params.max_linear_x, "max_linear_x"},
      {params.max_linear_y, "max_linear_y"},
      {params.max_linear_speed, "max_linear_speed"},
      {params.max_angular_z, "max_angular_z"}}};
  for (const auto & [value, name] : body_limits) {
    ValidatePositiveFinite(value, name);
  }

  // 축 하나만 최대로 줘도 바퀴 속도 상한을 넘으면 설정 오류
  const double required_linear_wheel_speed = params.max_linear_speed / params.wheel_radius;
  const double required_yaw_wheel_speed =
    (params.base_radius + std::hypot(params.kinematics_frame_x, params.kinematics_frame_y)) *
    params.max_angular_z / params.wheel_radius;
  if (required_linear_wheel_speed > params.max_wheel_speed ||
    required_yaw_wheel_speed > params.max_wheel_speed)
  {
    throw std::invalid_argument("individual body velocity limit exceeds max_wheel_speed");
  }

  ValidatePositiveFinite(params.encoder_read_period, "encoder_read_period");
  ValidatePositiveFinite(params.encoder_sample_timeout, "encoder_sample_timeout");
  if (params.encoder_feedback_source == "position") {
    if (params.encoder_position_tick_deadband != 0) {
      throw std::invalid_argument(
        "encoder_position_tick_deadband is deprecated and must be zero for position feedback");
    }
    // 3구간 창이 마감 시간 안에 들어와야 한다
    constexpr double timing_tolerance = 1e-12;
    if (3.0 * params.encoder_read_period >
      params.encoder_sample_timeout + timing_tolerance)
    {
      throw std::invalid_argument(
        "position feedback requires 3 * encoder_read_period <= encoder_sample_timeout");
    }
  }
}

void ValidateDriverMotorParameters(const BaseControllerParameters & params)
{
  ValidateWheelIdsParameter(params.wheel_ids);
  ValidateWheelDirectionsParameter(params.wheel_directions);

  if (params.encoder_feedback_source != "speed" && params.encoder_feedback_source != "position") {
    throw std::invalid_argument("encoder_feedback_source must be 'speed' or 'position'");
  }
  if (!std::isfinite(params.encoder_position_ticks_per_revolution) ||
    params.encoder_position_ticks_per_revolution <= 0.0 ||
    params.encoder_position_ticks_per_revolution >
    static_cast<double>(std::numeric_limits<int>::max()) ||
    std::trunc(params.encoder_position_ticks_per_revolution) !=
    params.encoder_position_ticks_per_revolution)
  {
    throw std::invalid_argument(
      "encoder_position_ticks_per_revolution must be a positive finite integer");
  }
  ValidatePositiveFinite(params.encoder_odom_scale, "encoder_odom_scale");
  if (params.encoder_position_tick_deadband < 0 ||
    params.encoder_position_tick_deadband >=
    static_cast<int64_t>(params.encoder_position_ticks_per_revolution / 2.0))
  {
    throw std::invalid_argument(
      "encoder_position_tick_deadband must be non-negative and less than half a revolution");
  }
  (void)IntFromParameter(params.encoder_position_tick_deadband, "encoder_position_tick_deadband");
  if (!std::isfinite(params.encoder_speed_deadband) || params.encoder_speed_deadband < 0.0) {
    throw std::invalid_argument("encoder_speed_deadband must be non-negative and finite");
  }

  ValidatePositiveFinite(params.max_wheel_speed, "max_wheel_speed");
  ValidatePositiveFinite(params.driver_states_timeout, "driver_states_timeout");
  ValidatePositiveFinite(params.torque_service_timeout, "torque_service_timeout");
}

std::array<uint8_t, WHEEL_COUNT> WheelIdsFromParameter(const std::vector<int64_t> & wheel_ids)
{
  ValidateWheelIdsParameter(wheel_ids);
  return {
    static_cast<uint8_t>(wheel_ids[0]),
    static_cast<uint8_t>(wheel_ids[1]),
    static_cast<uint8_t>(wheel_ids[2])};
}

std::array<double, WHEEL_COUNT> WheelDirectionsFromParameter(
  const std::vector<double> & wheel_directions)
{
  ValidateWheelDirectionsParameter(wheel_directions);
  return {wheel_directions[0], wheel_directions[1], wheel_directions[2]};
}

/*********************************************************
  명령 제한 · 오도메트리 선택
 *********************************************************/

geometry_msgs::msg::Twist LimitBodyTwist(
  const geometry_msgs::msg::Twist & command,
  const double max_linear_x,
  const double max_linear_y,
  const double max_linear_speed,
  const double max_angular_z,
  const OmniKinematics & kinematics)
{
  geometry_msgs::msg::Twist limited;
  if (!IsFiniteTwist(command)) {
    return limited;
  }

  // 축별 · 평면 속도 제한은 방향을 유지하도록 같은 비율로 줄인다
  double scale = 1.0;
  scale = std::min(scale, max_linear_x / std::max(std::abs(command.linear.x), 1e-12));
  scale = std::min(scale, max_linear_y / std::max(std::abs(command.linear.y), 1e-12));
  scale = std::min(
    scale,
    max_linear_speed / std::max(std::hypot(command.linear.x, command.linear.y), 1e-12));
  limited.linear.x = command.linear.x * scale;
  limited.linear.y = command.linear.y * scale;
  limited.angular.z = std::clamp(command.angular.z, -max_angular_z, max_angular_z);
  const BodyTwist effective = kinematics.ScaleToWheelEnvelope(
    BodyTwist{limited.linear.x, limited.linear.y, limited.angular.z});
  limited.linear.x = effective.linear_x;
  limited.linear.y = effective.linear_y;
  limited.angular.z = effective.angular_z;
  return limited;
}

geometry_msgs::msg::Twist SelectOdometryTwist(
  const geometry_msgs::msg::Twist & command,
  const bool has_command,
  const double command_age,
  const double command_timeout)
{
  if (!has_command || command_age > command_timeout) {
    return geometry_msgs::msg::Twist{};
  }
  return command;
}

geometry_msgs::msg::Twist SelectCommandAtTime(
  const geometry_msgs::msg::Twist & command,
  const bool has_command,
  const rclcpp::Time & last_command_time,
  const rclcpp::Time & current_time,
  const double command_timeout)
{
  return SelectOdometryTwist(
    command,
    has_command,
    (current_time - last_command_time).seconds(),
    command_timeout);
}

geometry_msgs::msg::Twist SelectOdometryUpdateTwist(
  const geometry_msgs::msg::Twist & command,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source,
  const OmniKinematics & kinematics)
{
  if (odom_source == "encoder") {
    if (!encoder_wheel_speeds.has_value()) {
      return geometry_msgs::msg::Twist{};
    }
    return ToRosTwist(kinematics.ToBodyTwist(*encoder_wheel_speeds));
  }
  return command;
}

void UpdateOdometryForSource(
  Odometry & odometry,
  const std::string & odom_source,
  const std::string & encoder_feedback_source,
  const geometry_msgs::msg::Twist & selected_twist,
  const double control_dt,
  const std::optional<WheelPositionDelta> & wheel_position_delta,
  const OmniKinematics & kinematics)
{
  if (odom_source == "encoder") {
    if (encoder_feedback_source == "position" && wheel_position_delta) {
      odometry.UpdateDelta(kinematics.ToBodyDelta(*wheel_position_delta));
    } else if (encoder_feedback_source == "speed") {
      odometry.Update(selected_twist, control_dt);
    }
    return;
  }
  odometry.Update(selected_twist, control_dt);
}

bool ShouldRefreshEncoderSample(
  const std::string & odom_source,
  const double encoder_sample_age,
  const double encoder_read_period)
{
  if (odom_source != "encoder") {
    return false;
  }
  return encoder_sample_age >= encoder_read_period;
}

std::optional<WheelCommand> SelectFreshEncoderWheelSpeeds(
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const double encoder_sample_age,
  const double encoder_sample_timeout)
{
  if (!encoder_wheel_speeds.has_value() || encoder_sample_age > encoder_sample_timeout) {
    return std::nullopt;
  }
  return encoder_wheel_speeds;
}

bool UpdateEncoderFeedbackOnSuccessfulRead(
  const std::optional<WheelFeedbackSample> & read_result,
  const rclcpp::Time & success_time,
  std::optional<WheelCommand> * last_sample,
  std::optional<rclcpp::Time> * last_success_time)
{
  if (!read_result || last_sample == nullptr || last_success_time == nullptr) {
    return false;
  }
  *last_sample = read_result->wheel_speeds;
  *last_success_time = success_time;
  return true;
}

bool IsEncoderFeedbackTimedOut(
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  const double encoder_sample_timeout)
{
  const auto & deadline_start = last_success_time ? last_success_time : encoder_wait_start_time;
  return deadline_start &&
         (current_time - *deadline_start).seconds() > encoder_sample_timeout;
}

bool IsMotorPowerEnableAllowed(
  const MotorReadinessLatch & motor_readiness,
  const bool is_recovery_disarmed)
{
  return !motor_readiness.IsFaultLatched() || is_recovery_disarmed;
}

bool StopMotorOnEncoderTimeout(
  BaseMotorInterface & motor_interface,
  MotorReadinessLatch & motor_readiness,
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  const double encoder_sample_timeout)
{
  if (!IsEncoderFeedbackTimedOut(
      encoder_wait_start_time,
      last_success_time,
      current_time,
      encoder_sample_timeout))
  {
    return false;
  }

  // 하나가 실패해도 다음 단계를 시도하고, 고장은 반드시 고정한다
  try {
    motor_interface.Stop();
  } catch (...) {
  }
  try {
    (void)motor_interface.SetMotorPower(false);
  } catch (...) {
  }
  motor_readiness.OnBackendFault();
  return true;
}

WheelCommand SelectJointStateWheelSpeeds(
  const WheelCommand & command_wheel_speeds,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source)
{
  if (odom_source == "encoder" && encoder_wheel_speeds.has_value()) {
    return *encoder_wheel_speeds;
  }
  return command_wheel_speeds;
}

void IntegrateWheelAngles(
  std::array<double, WHEEL_COUNT> * wheel_angles,
  const std::optional<WheelPositionDelta> & wheel_position_delta)
{
  if (wheel_angles == nullptr || !wheel_position_delta) {
    return;
  }
  const std::array<double, WHEEL_COUNT> deltas{
    wheel_position_delta->left, wheel_position_delta->back, wheel_position_delta->right};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    (*wheel_angles)[i] = std::remainder((*wheel_angles)[i] + deltas[i], TWO_PI);
  }
}

sensor_msgs::msg::JointState MakeBaseWheelJointState(
  const WheelCommand & wheel_speeds,
  const std::array<double, WHEEL_COUNT> & wheel_angles,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = {"left_wheel_joint", "back_wheel_joint", "right_wheel_joint"};
  msg.position = {wheel_angles[0], wheel_angles[1], wheel_angles[2]};
  msg.velocity = {wheel_speeds.left, wheel_speeds.back, wheel_speeds.right};
  return msg;
}

std_srvs::srv::Trigger::Response ResetOdometryForService(Odometry & odometry)
{
  odometry.Reset();

  std_srvs::srv::Trigger::Response response;
  response.success = true;
  response.message = "odometry reset";
  return response;
}

std_srvs::srv::SetBool::Response SetMotorPowerForService(
  BaseMotorInterface & motor_interface,
  const bool is_enabled)
{
  std_srvs::srv::SetBool::Response response;
  response.success = motor_interface.SetMotorPower(is_enabled);
  if (response.success) {
    response.message = is_enabled ? "motor power enabled" : "motor power disabled";
  } else {
    response.message =
      is_enabled ? "failed to enable motor power" : "failed to disable motor power";
  }
  return response;
}

rclcpp::Time OffsetTransformStamp(const rclcpp::Time & stamp, const double offset_seconds)
{
  return stamp + rclcpp::Duration::from_seconds(offset_seconds);
}

}  // namespace lekiwi_base_controller
