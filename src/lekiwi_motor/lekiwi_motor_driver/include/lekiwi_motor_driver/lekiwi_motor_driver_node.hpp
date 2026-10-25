#ifndef LEKIWI_MOTOR_DRIVER__LEKIWI_MOTOR_DRIVER_NODE_HPP_
#define LEKIWI_MOTOR_DRIVER__LEKIWI_MOTOR_DRIVER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <lekiwi_msgs/msg/arm_command.hpp>
#include <lekiwi_msgs/msg/arm_registers.hpp>
#include <lekiwi_msgs/msg/motor_state.hpp>
#include <lekiwi_msgs/msg/motor_state_array.hpp>
#include <lekiwi_msgs/msg/wheel_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "lekiwi_motor_driver/feetech_bus.hpp"

namespace lekiwi_motor_driver
{

// 바퀴·팔이 함께 쓰는 Feetech STS3215 버스를 여는 유일한 노드.
//   states_topic (기본 lekiwi/motor/states) : publish_rate_hz마다 모든 모터의 MotorState
// wheel_write_enabled가 true일 때만 바퀴 쓰기를 연다 (기본 false = 읽기 전용).
//   wheel_command_topic (기본 lekiwi/motor/wheel_command) : 바퀴 속도 명령
//   wheel_torque_service (기본 lekiwi/motor/wheel_torque) : 바퀴 토크 켜기 · 끄기
// arm_write_enabled가 true일 때만 팔 쓰기를 연다 (기본 false = 팔은 읽기만).
//   arm_command_topic (기본 lekiwi/motor/arm_command) : 팔 목표 위치 명령 [tick]
//   arm_torque_service (기본 lekiwi/motor/arm_torque) : 팔 토크 켜기 · 끄기
//   arm_ready_topic (기본 lekiwi/motor/arm_ready) : 팔 명령을 받는 중인지 (transient_local)
//   arm_registers_topic (기본 lekiwi/motor/arm_registers) : 팔 EEPROM 설정값 (transient_local, 읽기만)
// 쓰기는 wheel_ids · arm_ids에만 하나씩(또는 SYNC_WRITE로) 보낸다. 토크를 브로드캐스트로 쓰지 않는다.
// 모든 콜백(읽기 타이머, 바퀴 · 팔 명령, 토크 서비스)은 기본 콜백 그룹 하나에서 한 스레드로 차례로 돌아
// 버스 접근이 겹치지 않는다. 다른 콜백 그룹이나 MultiThreadedExecutor를 쓰지 않는다.
class LekiwiMotorDriverNode : public rclcpp::Node
{
public:
  LekiwiMotorDriverNode();
  ~LekiwiMotorDriverNode() override;

private:
  using Clock = std::chrono::steady_clock;

  // 응답이 막 끊긴 모터는 이만큼 주기마다 다시 읽는다. offline_after_failures(30 Hz 5번 약 167 ms)와 합쳐
  // 약 0.47 s 안의 끊김은 팔로워 팔 state_timeout_s(0.5 s) 전에 돌아온다. 그동안 그 모터마다 read_timeout_ms를 기다린다
  static constexpr std::chrono::milliseconds kOfflineRetryGrace{300};

  struct MotorSlot
  {
    lekiwi_msgs::msg::MotorState state;   // id, name과 이번 주기 읽기 결과
    bool responsive{false};               // false면 kOfflineRetryGrace 뒤로는 느린 읽기 차례에만 다시 시도한다
    int64_t consecutive_failures{0};
    uint8_t last_status{0};
    Clock::time_point offline_since{};    // responsive가 false가 된 시각 (시작할 때 없던 모터는 기본값)
  };

  // 파라미터 선언·로드
  void DeclareParams();
  void InitializeParams();

  // 시작할 때 모델 번호로 모터를 확인한다
  void ProbeMotors();

  // 주기 처리
  void TimerCallback();
  bool ReadPresent(MotorSlot & slot);
  bool ReadConfig(MotorSlot & slot);
  void HandleReadResult(MotorSlot & slot, bool ok);
  void ReportStatusChange(MotorSlot & slot);
  static void ClearReadings(lekiwi_msgs::msg::MotorState & state);

  // 바퀴 쓰기
  void CreateWheelInterfaces();
  void WheelCommandCallback(const lekiwi_msgs::msg::WheelCommand::SharedPtr msg);
  void WheelTorqueCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void SuperviseWheels();
  bool EnableWheelTorque(std::string & message);
  bool DisableWheelTorque(std::string & message);
  bool WriteZeroWheelSpeed();

  // 팔 쓰기
  void CreateArmInterfaces();
  void ArmCommandCallback(const lekiwi_msgs::msg::ArmCommand::SharedPtr msg);
  void ArmTorqueCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool EnableArmTorque(std::string & message);
  bool DisableArmTorque(std::string & message);
  // 팔마다 Goal_Position = 지금 위치(ACK) → 성공한 팔만 Torque_Enable 1(ACK). 실패한 ID를 돌려준다
  std::vector<uint8_t> WriteArmGoalAndTorque(const std::vector<MotorSlot *> & arms);
  void RetryArmTorqueOff();
  void SetArmReady(bool is_ready);
  bool ReadArmRegisters();   // 아직 읽지 못한 팔의 EEPROM 설정을 읽는다. 새로 읽으면 true

  // ids 모터에 하나씩 Torque_Enable을 쓰고 ACK를 확인한다. label은 메시지용 ("wheel", "arm")
  bool WriteTorque(
    const std::vector<uint8_t> & ids, const std::string & label, bool enable, std::string & message);
  // 위와 같이 쓰고 ACK가 없는(켤 때는 상태 바이트가 0이 아닌) ID를 돌려준다
  std::vector<uint8_t> WriteTorqueIds(const std::vector<uint8_t> & ids, bool enable);
  std::vector<MotorSlot *> MotorSlots(const std::vector<uint8_t> & ids);

  // 버스
  std::string port_;
  int baud_rate_{1000000};
  int read_timeout_ms_{10};
  std::unique_ptr<FeetechBus> bus_;
  std::vector<MotorSlot> slots_;

  // 주기 · 장애 처리
  double publish_rate_hz_{30.0};
  double slow_read_period_{1.0};
  int64_t offline_after_failures_{5};
  Clock::time_point last_slow_read_;      // 끄기 재시도 · 팔 EEPROM 다시 읽기 (slow_read_period마다)
  // 모터별 느린 읽기: slow_read_period / 모터 수마다 한 모터씩 차례로
  Clock::duration slow_slot_interval_{};
  Clock::time_point next_slow_slot_time_;
  std::size_t next_slow_slot_{0};

  std::string states_topic_;
  rclcpp::Publisher<lekiwi_msgs::msg::MotorStateArray>::SharedPtr pub_states_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 바퀴 쓰기 설정
  bool is_wheel_write_enabled_{false};
  std::vector<uint8_t> wheel_ids_;
  double speed_tick_scale_{651.8986469044033};   // [tick/s per rad/s] = 4096 / 2π
  int speed_tick_limit_{1500};                   // [tick/s]
  double wheel_command_timeout_{0.5};            // [s] 넘으면 속도 0
  double wheel_torque_off_timeout_{3.0};         // [s] 넘으면 토크 끔
  std::string wheel_command_topic_;
  std::string wheel_torque_service_;

  // 바퀴 쓰기 상태
  bool is_wheel_torque_on_{false};       // 이 노드가 토크를 켰고 아직 끄지 않았다
  bool has_pending_torque_off_{false};   // 끄기를 확인하지 못해 느린 주기마다 다시 보낸다
  bool is_command_stale_{false};         // 명령이 끊겨 속도 0을 쓰는 중 (로그 한 번만)
  Clock::time_point last_wheel_command_;
  rclcpp::Subscription<lekiwi_msgs::msg::WheelCommand>::SharedPtr sub_wheel_command_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_wheel_torque_;

  // 팔 쓰기 설정
  bool is_arm_write_enabled_{false};
  std::vector<uint8_t> arm_ids_;
  int arm_max_goal_offset_ticks_{1024};   // [tick] 목표는 마지막 Present_Position ± 이 값 안
  std::string arm_command_topic_;
  std::string arm_torque_service_;
  std::string arm_ready_topic_;
  std::string arm_registers_topic_;

  // 팔 쓰기 상태
  bool is_arm_ready_{false};                   // arm_torque true가 성공해 팔 명령을 받는다 (arm_ready)
  std::set<uint8_t> arm_pending_torque_off_;   // 끄기를 확인하지 못해 느린 주기마다 ID별로 다시 보낸다
  lekiwi_msgs::msg::ArmRegisters arm_registers_;
  rclcpp::Subscription<lekiwi_msgs::msg::ArmCommand>::SharedPtr sub_arm_command_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_arm_torque_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arm_ready_;
  rclcpp::Publisher<lekiwi_msgs::msg::ArmRegisters>::SharedPtr pub_arm_registers_;
};

}  // namespace lekiwi_motor_driver

#endif  // LEKIWI_MOTOR_DRIVER__LEKIWI_MOTOR_DRIVER_NODE_HPP_
