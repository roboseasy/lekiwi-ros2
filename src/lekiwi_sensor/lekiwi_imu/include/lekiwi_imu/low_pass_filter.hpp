#ifndef LEKIWI_IMU__LOW_PASS_FILTER_HPP_
#define LEKIWI_IMU__LOW_PASS_FILTER_HPP_

namespace lekiwi_imu
{

// 1차 저역 통과 필터 (지수 이동 평균): 출력 = alpha · 입력 + (1 - alpha) · 이전 출력.
// 첫 입력은 그대로 내보낸다. alpha가 1이면 거르지 않고, 0에 가까울수록 강하게 거른다.
class LowPassFilter
{
public:
  explicit LowPassFilter(double alpha = 1.0);

  // alpha는 (0, 1]. 범위 밖이면 std::invalid_argument
  void SetAlpha(double alpha);
  // 이전 출력을 잊는다. 다음 입력은 그대로 내보낸다.
  void Reset();

  double Filter(double input);
  // alpha를 바꾼 뒤 거른다 (One Euro처럼 샘플마다 alpha가 바뀔 때)
  double Filter(double input, double alpha);

  bool HasOutput() const;
  double LastOutput() const;

private:
  double alpha_{1.0};
  double last_output_{0.0};
  bool has_output_{false};
};

}  // namespace lekiwi_imu

#endif  // LEKIWI_IMU__LOW_PASS_FILTER_HPP_
