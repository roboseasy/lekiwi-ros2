#include "lekiwi_base_controller/motor_readiness.hpp"

namespace lekiwi_base_controller
{

void MotorReadinessLatch::OnPowerDisabled()
{
  state_ = MotorSafetyState::Disarmed;
}

bool MotorReadinessLatch::OnPowerEnabled(const bool is_hardware_connected)
{
  if (!is_hardware_connected) {
    state_ = MotorSafetyState::FaultLatched;
    return false;
  }
  if (state_ == MotorSafetyState::FaultLatched) {
    return false;
  }
  state_ = MotorSafetyState::WaitingForEncoder;
  return true;
}

void MotorReadinessLatch::OnFreshEncoderSample()
{
  if (state_ == MotorSafetyState::WaitingForEncoder) {
    state_ = MotorSafetyState::Ready;
  }
}

void MotorReadinessLatch::OnEncoderTimeout()
{
  if (state_ == MotorSafetyState::Ready) {
    state_ = MotorSafetyState::FaultLatched;
  }
}

void MotorReadinessLatch::OnBackendFault()
{
  state_ = MotorSafetyState::FaultLatched;
}

bool MotorReadinessLatch::IsReady(
  const bool is_real_hardware,
  const bool is_hardware_connected,
  const bool is_power_enabled) const
{
  return state_ == MotorSafetyState::Ready && is_real_hardware && is_hardware_connected &&
         is_power_enabled;
}

bool MotorReadinessLatch::IsArmed() const
{
  return state_ == MotorSafetyState::WaitingForEncoder || state_ == MotorSafetyState::Ready;
}

bool MotorReadinessLatch::IsFaultLatched() const
{
  return state_ == MotorSafetyState::FaultLatched;
}

bool MotorReadinessLatch::Permits(const WheelCommand & command) const
{
  const bool is_stopped = command.left == 0.0 && command.back == 0.0 && command.right == 0.0;
  return is_stopped || state_ == MotorSafetyState::Ready;
}

}  // namespace lekiwi_base_controller
