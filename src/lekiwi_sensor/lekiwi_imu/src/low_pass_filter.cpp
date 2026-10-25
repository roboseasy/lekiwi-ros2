#include "lekiwi_imu/low_pass_filter.hpp"

#include <stdexcept>
#include <string>

namespace lekiwi_imu
{

LowPassFilter::LowPassFilter(const double alpha)
{
  SetAlpha(alpha);
}

void LowPassFilter::SetAlpha(const double alpha)
{
  if (!(alpha > 0.0 && alpha <= 1.0)) {
    throw std::invalid_argument("low-pass alpha must be in (0, 1], got " + std::to_string(alpha));
  }
  alpha_ = alpha;
}

void LowPassFilter::Reset()
{
  last_output_ = 0.0;
  has_output_ = false;
}

double LowPassFilter::Filter(const double input)
{
  last_output_ = has_output_ ? alpha_ * input + (1.0 - alpha_) * last_output_ : input;
  has_output_ = true;
  return last_output_;
}

double LowPassFilter::Filter(const double input, const double alpha)
{
  SetAlpha(alpha);
  return Filter(input);
}

bool LowPassFilter::HasOutput() const
{
  return has_output_;
}

double LowPassFilter::LastOutput() const
{
  return last_output_;
}

}  // namespace lekiwi_imu
