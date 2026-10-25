#include "lekiwi_base_controller/motor_safety_coordinator.hpp"

#include <utility>

#include "lekiwi_base_controller/base_controller.hpp"

namespace lekiwi_base_controller
{

MotorSafetyCoordinator::MotorSafetyCoordinator(
  BaseMotorInterface & motor_interface,
  const bool is_real_hardware,
  const bool is_encoder_feedback_required,
  const double encoder_read_period,
  const double encoder_sample_timeout,
  const rclcpp::Time & initial_time,
  NowFunction now_function)
: motor_interface_(motor_interface),
  is_real_hardware_(is_real_hardware),
  is_encoder_feedback_required_(is_encoder_feedback_required),
  encoder_read_period_(encoder_read_period),
  encoder_sample_timeout_(encoder_sample_timeout),
  last_encoder_attempt_time_(initial_time),
  now_function_(std::move(now_function))
{
  if (is_real_hardware_ && motor_interface_.IsConnected() &&
    motor_interface_.IsMotorPowerEnabled())
  {
    if (motor_readiness_.OnPowerEnabled(true)) {
      encoder_wait_start_time_ = initial_time;
    }
  }
  is_last_ready_ = IsMotorReady();
}

MotorSafetyUpdate MotorSafetyCoordinator::BeforeCommand(const rclcpp::Time & attempt_time)
{
  const bool is_ready_before = is_last_ready_;
  MotorSafetyUpdate update;
  const auto finish = [this, is_ready_before](MotorSafetyUpdate result) {
      is_last_ready_ = IsMotorReady();
      result.is_readiness_changed = is_ready_before != is_last_ready_;
      return result;
    };

  const auto cycle_time = now_function_();
  update.safety_time = cycle_time;
  const auto sample_age = [this](const rclcpp::Time & time) {
      const auto & start = last_encoder_success_time_ ?
        last_encoder_success_time_ : encoder_wait_start_time_;
      return start ? (time - *start).seconds() : 0.0;
    };
  // (1) 읽기 전에 마감 시간을 먼저 확인한다
  if (is_encoder_feedback_required_ && motor_readiness_.IsArmed() && StopMotorOnEncoderTimeout(
      motor_interface_,
      motor_readiness_,
      encoder_wait_start_time_,
      last_encoder_success_time_,
      cycle_time,
      encoder_sample_timeout_))
  {
    is_recovery_disarmed_ = false;
    update.is_encoder_timeout = true;
    update.encoder_timeout_stage = EncoderTimeoutStage::BeforeRead;
    update.encoder_sample_age_seconds = sample_age(cycle_time);
    update.consecutive_failed_reads = consecutive_failed_reads_;
    return finish(update);
  }

  // (2) encoder_read_period마다 엔코더를 읽는다
  if (is_encoder_feedback_required_ &&
    last_encoder_attempt_time_ &&
    (attempt_time - *last_encoder_attempt_time_).seconds() >= encoder_read_period_)
  {
    last_encoder_attempt_time_ = attempt_time;
    std::optional<WheelFeedbackSample> read_result;
    try {
      read_result = motor_interface_.ReadWheelFeedback();
    } catch (...) {
      update.encoder_read_status = EncoderReadStatus::Exception;
    }
    if (update.encoder_read_status != EncoderReadStatus::Exception) {
      update.encoder_read_status = read_result ?
        EncoderReadStatus::Sample : EncoderReadStatus::NoSample;
    }
    const auto completion_time = now_function_();
    update.safety_time = completion_time;
    update.encoder_read_duration_seconds = (completion_time - cycle_time).seconds();
    if (!read_result) {
      ++consecutive_failed_reads_;
    }
    // (3) 읽는 동안 마감 시간을 넘겼으면 이번 샘플은 버린다
    if (motor_readiness_.IsArmed() && StopMotorOnEncoderTimeout(
        motor_interface_,
        motor_readiness_,
        encoder_wait_start_time_,
        last_encoder_success_time_,
        completion_time,
        encoder_sample_timeout_))
    {
      is_recovery_disarmed_ = false;
      update.is_encoder_timeout = true;
      update.encoder_timeout_stage = EncoderTimeoutStage::AfterRead;
      update.encoder_sample_age_seconds = sample_age(completion_time);
      update.consecutive_failed_reads = consecutive_failed_reads_;
      return finish(update);
    }
    if (UpdateEncoderFeedbackOnSuccessfulRead(
        read_result,
        completion_time,
        &last_encoder_wheel_speeds_,
        &last_encoder_success_time_))
    {
      update.wheel_position_delta = read_result->wheel_position_delta;
      consecutive_failed_reads_ = 0;
      encoder_wait_start_time_.reset();
      motor_readiness_.OnFreshEncoderSample();
    }
  }

  // (4) 연결이 끊겼거나 전원이 꺼졌으면 고장 고정
  if (motor_readiness_.IsArmed() &&
    (!motor_interface_.IsConnected() || !motor_interface_.IsMotorPowerEnabled()))
  {
    motor_readiness_.OnBackendFault();
    is_recovery_disarmed_ = false;
  }
  return finish(update);
}

void MotorSafetyCoordinator::SendWheelCommand(const WheelCommand & command)
{
  const bool is_armed_before_write = motor_readiness_.IsArmed();
  motor_interface_.WriteWheelSpeeds(motor_readiness_.Permits(command) ? command : WheelCommand{});
  if (is_armed_before_write &&
    (!motor_interface_.IsConnected() || !motor_interface_.IsMotorPowerEnabled()))
  {
    motor_readiness_.OnBackendFault();
    is_recovery_disarmed_ = false;
  }
}

std::optional<WheelCommand> MotorSafetyCoordinator::GetFreshEncoderWheelSpeeds(
  const rclcpp::Time & current_time) const
{
  if (!last_encoder_success_time_) {
    return std::nullopt;
  }
  return SelectFreshEncoderWheelSpeeds(
    last_encoder_wheel_speeds_,
    (current_time - *last_encoder_success_time_).seconds(),
    encoder_sample_timeout_);
}

std_srvs::srv::SetBool::Response MotorSafetyCoordinator::SetMotorPower(
  const bool is_enabled,
  const rclcpp::Time & request_time)
{
  const auto finish = [this](std_srvs::srv::SetBool::Response response) {
      is_last_ready_ = IsMotorReady();
      return response;
    };
  if (!is_real_hardware_) {
    return finish(SetMotorPowerForService(motor_interface_, is_enabled));
  }

  std_srvs::srv::SetBool::Response response;
  if (!is_enabled) {
    const bool is_success = motor_interface_.SetMotorPower(false);
    if (is_success) {
      motor_readiness_.OnPowerDisabled();
      is_recovery_disarmed_ = true;
      encoder_wait_start_time_.reset();
      last_encoder_wheel_speeds_.reset();
      last_encoder_success_time_.reset();
      last_encoder_attempt_time_.reset();
      consecutive_failed_reads_ = 0;
    } else {
      motor_readiness_.OnBackendFault();
      is_recovery_disarmed_ = false;
    }
    response.success = is_success;
    response.message = is_success ? "motor power disabled" : "failed to disable motor power";
    return finish(response);
  }

  if (!IsMotorPowerEnableAllowed(motor_readiness_, is_recovery_disarmed_)) {
    response.success = false;
    response.message = "disable motor power before recovering a latched fault";
    return finish(response);
  }

  if (!motor_interface_.IsConnected()) {
    motor_interface_.Connect();
  }
  if (!motor_interface_.IsConnected() || !motor_interface_.SetMotorPower(true)) {
    motor_interface_.Stop();
    (void)motor_interface_.SetMotorPower(false);
    motor_readiness_.OnBackendFault();
    is_recovery_disarmed_ = false;
    response.success = false;
    response.message = "failed to connect and enable motor power";
    return finish(response);
  }

  if (!motor_readiness_.OnPowerEnabled(true)) {
    is_recovery_disarmed_ = false;
    response.success = false;
    response.message = "failed to arm encoder readiness check";
    return finish(response);
  }

  is_recovery_disarmed_ = false;
  encoder_wait_start_time_ = request_time;
  last_encoder_wheel_speeds_.reset();
  last_encoder_success_time_.reset();
  consecutive_failed_reads_ = 0;
  last_encoder_attempt_time_ = request_time;
  response.success = true;
  response.message = "motor power enabled; waiting for fresh encoder feedback";
  return finish(response);
}

bool MotorSafetyCoordinator::IsMotorReady() const
{
  return motor_readiness_.IsReady(
    is_real_hardware_,
    motor_interface_.IsConnected(),
    motor_interface_.IsMotorPowerEnabled());
}

bool MotorSafetyCoordinator::IsFaultLatched() const
{
  return motor_readiness_.IsFaultLatched();
}

rclcpp::QoS MakeMotorReadyQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace lekiwi_base_controller
