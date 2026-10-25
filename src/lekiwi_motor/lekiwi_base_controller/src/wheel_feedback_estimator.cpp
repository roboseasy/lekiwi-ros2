#include "lekiwi_base_controller/wheel_feedback_estimator.hpp"

#include <cmath>
#include <utility>

namespace lekiwi_base_controller
{
namespace
{
constexpr double TWO_PI = 6.28318530717958647692;
}  // namespace

int WrapPositionDelta(
  const int current_ticks,
  const int previous_ticks,
  const int ticks_per_revolution)
{
  int delta = current_ticks - previous_ticks;
  const int half_revolution = ticks_per_revolution / 2;
  if (delta > half_revolution) {
    delta -= ticks_per_revolution;
  } else if (delta < -half_revolution) {
    delta += ticks_per_revolution;
  }
  return delta;
}

std::array<double, WHEEL_COUNT> ToMotorVelocities(
  const WheelCommand & command,
  const std::array<double, WHEEL_COUNT> & wheel_directions)
{
  return {
    command.left * wheel_directions[0],
    command.back * wheel_directions[1],
    command.right * wheel_directions[2]};
}

WheelFeedbackEstimator::WheelFeedbackEstimator(WheelFeedbackConfig config)
: config_(std::move(config))
{
}

void WheelFeedbackEstimator::Reset()
{
  position_intervals_.clear();
  last_position_ticks_.reset();
  last_position_stamp_nanoseconds_.reset();
}

const WheelFeedbackDiagnostics & WheelFeedbackEstimator::GetLastDiagnostics() const
{
  return last_diagnostics_;
}

const WheelFeedbackConfig & WheelFeedbackEstimator::GetConfig() const
{
  return config_;
}

std::optional<WheelFeedbackSample> WheelFeedbackEstimator::Update(const WheelStatesFrame & frame)
{
  // 바퀴 하나라도 읽기에 실패했으면 이번 읽기 실패로 본다
  for (const bool is_online : frame.is_online) {
    if (!is_online) {
      return std::nullopt;
    }
  }
  if (config_.source == EncoderFeedbackSource::Position) {
    return UpdatePosition(frame);
  }
  return UpdateSpeed(frame);
}

std::optional<WheelFeedbackSample> WheelFeedbackEstimator::UpdatePosition(
  const WheelStatesFrame & frame)
{
  const int ticks_per_revolution = config_.position_ticks_per_revolution;
  for (const int ticks : frame.position_ticks) {
    if (ticks < 0 || ticks >= ticks_per_revolution) {
      return std::nullopt;
    }
  }

  // 첫 샘플: 기준점만 잡고, 창은 encoder_read_period 길이의 0 구간 두 개로 채운다
  if (!last_position_ticks_) {
    last_position_ticks_ = frame.position_ticks;
    last_position_stamp_nanoseconds_ = frame.stamp_nanoseconds;
    position_intervals_.push_back(PositionInterval{{0, 0, 0}, config_.encoder_read_period});
    position_intervals_.push_back(PositionInterval{{0, 0, 0}, config_.encoder_read_period});
    return std::nullopt;
  }

  const double elapsed_seconds =
    static_cast<double>(frame.stamp_nanoseconds - *last_position_stamp_nanoseconds_) * 1.0e-9;
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
    return std::nullopt;
  }

  PositionInterval current_interval;
  current_interval.elapsed_seconds = elapsed_seconds;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    current_interval.delta_ticks[i] = WrapPositionDelta(
      frame.position_ticks[i], (*last_position_ticks_)[i], ticks_per_revolution);
  }

  auto updated_intervals = position_intervals_;
  updated_intervals.push_back(current_interval);
  while (updated_intervals.size() > 3u) {
    updated_intervals.pop_front();
  }

  std::array<int, WHEEL_COUNT> window_delta_ticks{};
  double window_elapsed_seconds = 0.0;
  for (const auto & interval : updated_intervals) {
    window_elapsed_seconds += interval.elapsed_seconds;
    for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
      window_delta_ticks[i] += interval.delta_ticks[i];
    }
  }
  if (!std::isfinite(window_elapsed_seconds) || window_elapsed_seconds <= 0.0) {
    return std::nullopt;
  }

  std::array<double, WHEEL_COUNT> wheel_speeds{};
  std::array<double, WHEEL_COUNT> wheel_position_deltas{};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    const double direction_and_scale =
      config_.encoder_odom_scale / config_.wheel_directions[i];
    wheel_speeds[i] =
      static_cast<double>(window_delta_ticks[i]) * TWO_PI /
      static_cast<double>(ticks_per_revolution) /
      window_elapsed_seconds * direction_and_scale;
    wheel_position_deltas[i] =
      static_cast<double>(current_interval.delta_ticks[i]) * TWO_PI /
      static_cast<double>(ticks_per_revolution) * direction_and_scale;
    if (!std::isfinite(wheel_speeds[i]) || !std::isfinite(wheel_position_deltas[i])) {
      return std::nullopt;
    }
  }

  last_diagnostics_.previous_ticks = *last_position_ticks_;
  last_diagnostics_.delta_ticks = current_interval.delta_ticks;
  last_diagnostics_.window_delta_ticks = window_delta_ticks;
  last_diagnostics_.window_elapsed_seconds = window_elapsed_seconds;

  position_intervals_ = std::move(updated_intervals);
  last_position_ticks_ = frame.position_ticks;
  last_position_stamp_nanoseconds_ = frame.stamp_nanoseconds;
  return WheelFeedbackSample{
    WheelCommand{wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]},
    WheelPositionDelta{
      wheel_position_deltas[0], wheel_position_deltas[1], wheel_position_deltas[2]}};
}

std::optional<WheelFeedbackSample> WheelFeedbackEstimator::UpdateSpeed(
  const WheelStatesFrame & frame) const
{
  std::array<double, WHEEL_COUNT> wheel_speeds{};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    const double raw_velocity = frame.velocity[i];
    if (!std::isfinite(raw_velocity)) {
      return std::nullopt;
    }
    const double filtered_velocity =
      std::abs(raw_velocity) <= config_.encoder_speed_deadband ? 0.0 : raw_velocity;
    const double raw_wheel_speed = filtered_velocity / config_.wheel_directions[i];
    wheel_speeds[i] = raw_wheel_speed * config_.encoder_odom_scale;
  }
  return WheelFeedbackSample{
    WheelCommand{wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]}, std::nullopt};
}

}  // namespace lekiwi_base_controller
