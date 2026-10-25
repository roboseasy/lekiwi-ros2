#ifndef LEKIWI_BASE_CONTROLLER__WHEEL_FEEDBACK_ESTIMATOR_HPP_
#define LEKIWI_BASE_CONTROLLER__WHEEL_FEEDBACK_ESTIMATOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "lekiwi_base_controller/omni_kinematics.hpp"

// ROS에 의존하지 않는 바퀴 엔코더 계산 (단위 테스트 대상).
// 입력은 모터 드라이버 states 한 메시지에서 설정된 바퀴 3개만 모델 순서(left, back, right)로 뽑은 값이다.
namespace lekiwi_base_controller
{

constexpr std::size_t WHEEL_COUNT = 3;

// 엔코더 한 번 읽기 결과 (모델 관절 기준, 방향 · 보정 적용 후)
struct WheelFeedbackSample
{
  WheelCommand wheel_speeds{};                            // [rad/s]
  std::optional<WheelPositionDelta> wheel_position_delta;  // position 피드백일 때만
};

enum class EncoderFeedbackSource
{
  Position,  // position_ticks 차분 (3구간 창 속도 + 이번 구간 위치 증분)
  Speed,     // 모터가 보고한 velocity
};

// states 메시지 한 개에서 뽑은 바퀴 3개 값
struct WheelStatesFrame
{
  int64_t stamp_nanoseconds{0};            // MotorStateArray header.stamp
  std::array<bool, WHEEL_COUNT> is_online{};
  std::array<int, WHEEL_COUNT> position_ticks{};
  std::array<double, WHEEL_COUNT> velocity{};  // [rad/s] 모터 축 기준
};

struct WheelFeedbackConfig
{
  EncoderFeedbackSource source{EncoderFeedbackSource::Position};
  std::array<double, WHEEL_COUNT> wheel_directions{1.0, 1.0, 1.0};
  int position_ticks_per_revolution{4096};
  double encoder_odom_scale{1.0};
  double encoder_read_period{0.1};      // [s] 첫 샘플 뒤 창을 채우는 가상 구간 길이
  double encoder_speed_deadband{50.0 / 651.8986469044033};  // [rad/s] speed 피드백 불감대
};

// 로그용 계산 중간값
struct WheelFeedbackDiagnostics
{
  std::array<int, WHEEL_COUNT> previous_ticks{};
  std::array<int, WHEEL_COUNT> delta_ticks{};
  std::array<int, WHEEL_COUNT> window_delta_ticks{};
  double window_elapsed_seconds{0.0};
};

// 바퀴 위치 · 속도 추정기
//   position: 처음 샘플은 기준점만 잡고 nullopt. 이후 반 바퀴 기준으로 감은 차분을 쓰고,
//             속도 = 최근 3구간 tick 합 / 3구간 시간, 위치 증분 = 이번 구간 차분
//   speed: velocity / 방향 × encoder_odom_scale, |velocity| <= 불감대이면 0
//   바퀴 하나라도 offline이면 nullopt (기준점 · 창 유지)
class WheelFeedbackEstimator
{
public:
  explicit WheelFeedbackEstimator(WheelFeedbackConfig config);

  std::optional<WheelFeedbackSample> Update(const WheelStatesFrame & frame);
  void Reset();
  const WheelFeedbackDiagnostics & GetLastDiagnostics() const;
  const WheelFeedbackConfig & GetConfig() const;

private:
  struct PositionInterval
  {
    std::array<int, WHEEL_COUNT> delta_ticks{};
    double elapsed_seconds{0.0};
  };

  std::optional<WheelFeedbackSample> UpdatePosition(const WheelStatesFrame & frame);
  std::optional<WheelFeedbackSample> UpdateSpeed(const WheelStatesFrame & frame) const;

  WheelFeedbackConfig config_;
  std::deque<PositionInterval> position_intervals_;
  std::optional<std::array<int, WHEEL_COUNT>> last_position_ticks_;
  std::optional<int64_t> last_position_stamp_nanoseconds_;
  WheelFeedbackDiagnostics last_diagnostics_;
};

// 이전 → 현재 tick 차분을 ±반 바퀴 안으로 감는다
int WrapPositionDelta(int current_ticks, int previous_ticks, int ticks_per_revolution);

// 모델 순서 바퀴 속도 → 모터 축 속도 (속도 × 방향)
std::array<double, WHEEL_COUNT> ToMotorVelocities(
  const WheelCommand & command,
  const std::array<double, WHEEL_COUNT> & wheel_directions);

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__WHEEL_FEEDBACK_ESTIMATOR_HPP_
