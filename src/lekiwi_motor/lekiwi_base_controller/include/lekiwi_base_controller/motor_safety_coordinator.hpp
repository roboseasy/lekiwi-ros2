#ifndef LEKIWI_BASE_CONTROLLER__MOTOR_SAFETY_COORDINATOR_HPP_
#define LEKIWI_BASE_CONTROLLER__MOTOR_SAFETY_COORDINATOR_HPP_

#include <functional>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "lekiwi_base_controller/motor_readiness.hpp"

namespace lekiwi_base_controller
{

class BaseMotorInterface;

enum class EncoderReadStatus { NotAttempted, Sample, NoSample, Exception };
enum class EncoderTimeoutStage { None, BeforeRead, AfterRead };

// 제어 주기 한 번의 안전 판단 결과
struct MotorSafetyUpdate
{
  rclcpp::Time safety_time{0, 0};
  bool is_readiness_changed{false};
  bool is_encoder_timeout{false};
  EncoderTimeoutStage encoder_timeout_stage{EncoderTimeoutStage::None};
  EncoderReadStatus encoder_read_status{EncoderReadStatus::NotAttempted};
  double encoder_sample_age_seconds{0.0};
  double encoder_read_duration_seconds{0.0};
  unsigned int consecutive_failed_reads{0};
  std::optional<WheelPositionDelta> wheel_position_delta;
};

// 엔코더 신선도 · 모터 전원 · 준비 래치를 묶은 안전 상태기계.
//   엔코더가 encoder_sample_timeout 넘게 끊기면 속도 0 → 전원 끔 → 고장 고정
//   고장은 SetMotorPower(false) 성공 후에만 다시 켤 수 있다
class MotorSafetyCoordinator
{
public:
  using NowFunction = std::function<rclcpp::Time()>;

  MotorSafetyCoordinator(
    BaseMotorInterface & motor_interface,
    bool is_real_hardware,
    bool is_encoder_feedback_required,
    double encoder_read_period,
    double encoder_sample_timeout,
    const rclcpp::Time & initial_time,
    NowFunction now_function);

  MotorSafetyUpdate BeforeCommand(const rclcpp::Time & attempt_time);
  void SendWheelCommand(const WheelCommand & command);
  std::optional<WheelCommand> GetFreshEncoderWheelSpeeds(const rclcpp::Time & current_time) const;
  std_srvs::srv::SetBool::Response SetMotorPower(
    bool is_enabled,
    const rclcpp::Time & request_time);
  bool IsMotorReady() const;
  bool IsFaultLatched() const;

private:
  BaseMotorInterface & motor_interface_;
  bool is_real_hardware_;
  bool is_encoder_feedback_required_;
  double encoder_read_period_;
  double encoder_sample_timeout_;
  std::optional<rclcpp::Time> encoder_wait_start_time_;
  std::optional<rclcpp::Time> last_encoder_attempt_time_;
  std::optional<rclcpp::Time> last_encoder_success_time_;
  std::optional<WheelCommand> last_encoder_wheel_speeds_;
  unsigned int consecutive_failed_reads_{0};
  bool is_recovery_disarmed_{false};
  bool is_last_ready_{false};
  MotorReadinessLatch motor_readiness_;
  NowFunction now_function_;
};

// lekiwi/motor/ready: reliable + transient_local, 깊이 1
rclcpp::QoS MakeMotorReadyQos();

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__MOTOR_SAFETY_COORDINATOR_HPP_
