#include "lekiwi_imu/one_euro_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace lekiwi_imu
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

void OneEuroFilter::Configure(const OneEuroParameters & parameters)
{
  if (!(parameters.rate_hz > 0.0)) {
    throw std::invalid_argument("one euro rate_hz must be positive");
  }
  if (!(parameters.min_cutoff_hz > 0.0)) {
    throw std::invalid_argument("one euro min_cutoff_hz must be positive");
  }
  if (!(parameters.derivative_cutoff_hz > 0.0)) {
    throw std::invalid_argument("one euro derivative_cutoff_hz must be positive");
  }
  parameters_ = parameters;
  rate_hz_ = parameters.rate_hz;
  value_filter_.Reset();
  speed_filter_.Reset();
  last_stamp_sec_.reset();
}

double OneEuroFilter::SmoothingFactor(const double cutoff_hz) const
{
  // alpha = 1 / (1 + tau / Te), tau = 1 / (2π · cutoff), Te = 1 / rate
  return 1.0 / (1.0 + rate_hz_ / (kTwoPi * cutoff_hz));
}

double OneEuroFilter::Filter(const double input, const std::optional<double> stamp_sec)
{
  if (stamp_sec && last_stamp_sec_ && *stamp_sec > *last_stamp_sec_) {
    rate_hz_ = 1.0 / (*stamp_sec - *last_stamp_sec_);
  }
  last_stamp_sec_ = stamp_sec;

  // 직전 출력과의 차이로 초당 변화량을 구하고, 그 크기만큼 차단 주파수를 올린다.
  const double speed = value_filter_.HasOutput() ? (input - value_filter_.LastOutput()) * rate_hz_ : 0.0;
  const double smoothed_speed =
    speed_filter_.Filter(speed, SmoothingFactor(parameters_.derivative_cutoff_hz));
  const double cutoff_hz = parameters_.min_cutoff_hz + parameters_.beta * std::fabs(smoothed_speed);
  return value_filter_.Filter(input, SmoothingFactor(cutoff_hz));
}

}  // namespace lekiwi_imu
