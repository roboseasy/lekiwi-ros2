#ifndef LEKIWI_IMU__ONE_EURO_FILTER_HPP_
#define LEKIWI_IMU__ONE_EURO_FILTER_HPP_

#include <optional>

#include "lekiwi_imu/low_pass_filter.hpp"

namespace lekiwi_imu
{

struct OneEuroParameters
{
  double rate_hz{100.0};             // 샘플 주파수 추정값 (> 0). 시각을 주면 샘플 간격으로 갱신한다
  double min_cutoff_hz{1.0};         // 최소 차단 주파수 (> 0). 작을수록 떨림을 더 줄인다
  double beta{0.0};                  // 변화 속도만큼 차단 주파수를 올리는 비율. 클수록 빠른 변화를 빨리 따라간다
  double derivative_cutoff_hz{1.0};  // 변화 속도를 거르는 차단 주파수 (> 0)
};

// One Euro 필터 (Casiez · Roussel · Vogel, CHI 2012). 천천히 변할 때는 떨림을 강하게 줄이고,
// 빠르게 변할 때는 차단 주파수를 올려 지연을 줄인다.
class OneEuroFilter
{
public:
  // 값을 확인하고(범위 밖이면 std::invalid_argument) 이전 상태를 지운다.
  void Configure(const OneEuroParameters & parameters);

  // stamp_sec: 샘플 시각 [s]. 앞 샘플보다 뒤면 그 간격으로 주파수를 갱신하고, 없으면 마지막 주파수를 쓴다.
  double Filter(double input, std::optional<double> stamp_sec = std::nullopt);

private:
  // 차단 주파수 cutoff_hz인 1차 필터의 alpha (지금 샘플 주파수 기준)
  double SmoothingFactor(double cutoff_hz) const;

  OneEuroParameters parameters_;
  double rate_hz_{100.0};
  LowPassFilter value_filter_;
  LowPassFilter speed_filter_;
  std::optional<double> last_stamp_sec_;
};

}  // namespace lekiwi_imu

#endif  // LEKIWI_IMU__ONE_EURO_FILTER_HPP_
