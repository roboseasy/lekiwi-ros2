#ifndef LEKIWI_BASE_CONTROLLER__MOTOR_READINESS_HPP_
#define LEKIWI_BASE_CONTROLLER__MOTOR_READINESS_HPP_

#include "lekiwi_base_controller/omni_kinematics.hpp"

namespace lekiwi_base_controller
{

enum class MotorSafetyState
{
  Disarmed,           // 모터 전원 꺼짐. 속도 0만 허용
  WaitingForEncoder,  // 전원은 켰고 새 엔코더 샘플을 기다리는 중
  Ready,              // 움직임 허용
  FaultLatched,       // 고장 고정. 전원 끄기 후에만 해제
};

// 모터 준비 상태 래치. 고장은 OnPowerDisabled() 전까지 풀리지 않는다.
class MotorReadinessLatch
{
public:
  void OnPowerDisabled();
  bool OnPowerEnabled(bool is_hardware_connected);
  void OnFreshEncoderSample();
  void OnEncoderTimeout();
  void OnBackendFault();
  bool IsReady(bool is_real_hardware, bool is_hardware_connected, bool is_power_enabled) const;
  bool IsArmed() const;
  bool IsFaultLatched() const;
  // Ready가 아니면 속도 0 명령만 허용한다
  bool Permits(const WheelCommand & command) const;

private:
  MotorSafetyState state_{MotorSafetyState::Disarmed};
};

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__MOTOR_READINESS_HPP_
