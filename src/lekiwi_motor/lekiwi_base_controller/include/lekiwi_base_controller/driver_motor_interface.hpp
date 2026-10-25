#ifndef LEKIWI_BASE_CONTROLLER__DRIVER_MOTOR_INTERFACE_HPP_
#define LEKIWI_BASE_CONTROLLER__DRIVER_MOTOR_INTERFACE_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <lekiwi_msgs/msg/motor_state_array.hpp>
#include <lekiwi_msgs/msg/wheel_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "lekiwi_base_controller/base_controller.hpp"
#include "lekiwi_base_controller/wheel_feedback_estimator.hpp"

namespace lekiwi_base_controller
{

struct DriverMotorConfig
{
  // 모델 순서 (left, back, right)
  std::array<uint8_t, WHEEL_COUNT> wheel_ids{8, 9, 7};
  double max_wheel_speed{2.3009711818284617};
  WheelFeedbackConfig feedback{};       // wheel_directions는 명령 변환에도 쓴다
  std::string states_topic{"lekiwi/motor/states"};
  std::string wheel_command_topic{"lekiwi/motor/wheel_command"};
  std::string wheel_torque_service{"lekiwi/motor/wheel_torque"};
  double driver_states_timeout{0.5};    // [s]
  double torque_service_timeout{1.0};   // [s]
  bool is_encoder_read_logged{false};
};

// states 한 메시지에서 설정된 바퀴 3개를 모델 순서로 뽑는다. 메시지에 없는 ID는 offline으로 본다
WheelStatesFrame ExtractWheelStatesFrame(
  const lekiwi_msgs::msg::MotorStateArray & states,
  const std::array<uint8_t, WHEEL_COUNT> & wheel_ids);

// 검증된 노드 파라미터 → 백엔드 설정
DriverMotorConfig MakeDriverMotorConfig(const BaseControllerParameters & params);

// 모델 순서 바퀴 속도 → WheelCommand (ids = wheel_ids, velocity = 속도 × 방향)
lekiwi_msgs::msg::WheelCommand MakeWheelCommandMessage(
  const WheelCommand & command,
  const std::array<uint8_t, WHEEL_COUNT> & wheel_ids,
  const std::array<double, WHEEL_COUNT> & wheel_directions,
  const rclcpp::Time & stamp);

// lekiwi_motor_driver_node를 ROS로 쓰는 바퀴 백엔드. 시리얼 포트를 열지 않는다.
//   states_topic         구독: 엔코더 · 연결 상태
//   wheel_command_topic  발행: 바퀴 속도 명령
//   wheel_torque_service 호출: 토크 켜기 · 끄기 (동기 호출, 최대 torque_service_timeout)
// states 구독과 토크 클라이언트는 각자 상호배타 콜백 그룹을 쓴다. 그래서 제어 그룹(타이머 · 서비스)
// 안에서 토크 서비스를 동기 호출해도 MultiThreadedExecutor의 다른 스레드가 응답을 처리한다.
//
// 연결이 끊긴 것(IsConnected() false)을 토크가 켜진 동안 발견하면 이 백엔드가 직접
// 속도 0 → 토크 끔을 보낸다. 끄기를 확인하지 못하면 명령 발행을 멈춰 드라이버 watchdog에 맡긴다.
// 드라이버가 스스로 토크를 끈 것(states의 torque_enabled=false)도 연결 끊김과 같이 처리한다.
class DriverMotorInterface final : public BaseMotorInterface
{
public:
  // 켜기 성공 후 이 시간 [s]보다 늦게 찍힌 states부터 torque_enabled를 본다
  // (켜기 직전에 발행된 메시지가 서비스 응답보다 늦게 도착할 수 있다)
  static constexpr double torque_guard_seconds = 0.1;

  DriverMotorInterface(rclcpp::Node & node, DriverMotorConfig config);

  void Connect() override;
  void Shutdown() noexcept override;
  // 최근 states가 driver_states_timeout 안에 왔고, 바퀴마다 online=true로 본 마지막 시각도
  // driver_states_timeout 안이면 true. 한 메시지의 일시적 offline은 연결 끊김이 아니다.
  // 토크가 켜진 동안 드라이버가 바퀴 torque_enabled=false를 보고하면 다음 전원 요청까지 false
  bool IsConnected() const override;
  // 마지막으로 쓴 메시지보다 header.stamp가 새로운 states가 있을 때만 샘플을 만든다
  std::optional<WheelFeedbackSample> ReadWheelFeedback() override;
  bool SetMotorPower(bool is_enabled) override;

private:
  void WriteClampedWheelSpeeds(const WheelCommand & command) override;
  void StatesCallback(const lekiwi_msgs::msg::MotorStateArray::SharedPtr msg);
  void PublishWheelCommand(const WheelCommand & command);
  bool CallWheelTorque(bool is_enabled);
  void HandleLinkLoss();
  void SetCommandBlocked(bool is_blocked);
  void SetTorqueGuard(std::optional<int64_t> guard_stamp_nanoseconds, bool is_clearing_torque_off);
  void LogEncoderRead(
    const WheelStatesFrame & frame,
    const std::optional<WheelFeedbackSample> & sample) const;
  std::string DescribeLink() const;

  DriverMotorConfig config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  WheelFeedbackEstimator estimator_;

  rclcpp::CallbackGroup::SharedPtr states_group_;
  rclcpp::CallbackGroup::SharedPtr torque_client_group_;
  rclcpp::Subscription<lekiwi_msgs::msg::MotorStateArray>::SharedPtr sub_states_;
  rclcpp::Publisher<lekiwi_msgs::msg::WheelCommand>::SharedPtr pub_wheel_command_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_client_;

  // states 콜백 스레드와 공유
  mutable std::mutex states_mutex_;
  lekiwi_msgs::msg::MotorStateArray::SharedPtr latest_states_;
  std::optional<rclcpp::Time> last_states_time_;
  std::array<std::optional<rclcpp::Time>, WHEEL_COUNT> last_online_times_;
  // 토크가 켜진 동안만 값이 있다. 이보다 늦게 찍힌 states에서 torque_enabled를 확인한다
  std::optional<int64_t> torque_guard_stamp_nanoseconds_;
  std::array<bool, WHEEL_COUNT> is_driver_torque_off_{};

  // 종료 콜백 스레드와 공유. 토크 끄기를 확인하지 못했거나 종료 중이면 명령을 보내지 않는다
  std::mutex command_mutex_;
  bool is_command_blocked_{false};

  // 제어 그룹에서만 쓴다
  std::optional<int64_t> last_consumed_stamp_nanoseconds_;
  bool is_link_loss_handled_{false};
};

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__DRIVER_MOTOR_INTERFACE_HPP_
