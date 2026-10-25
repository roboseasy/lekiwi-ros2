#ifndef LEKIWI_IMU__KALMAN_FILTER_HPP_
#define LEKIWI_IMU__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace lekiwi_imu
{

// 선형 칼만 필터. 이 노드에서는 전이 · 관측 행렬을 단위 행렬로 두고
// 가속도 3축 + 각속도 3축(상태 6개)을 평활화한다.
class KalmanFilter
{
public:
  // 상태 0, 오차 공분산 I. 프로세스 · 측정 노이즈 공분산은 단위 행렬에 각 값을 곱한 것이다.
  void Initialize(int state_size, int measurement_size, double process_noise, double measurement_noise);
  void SetState(const Eigen::VectorXd & state);
  // 예측: x ← F x, P ← F P Fᵀ + Q
  void Predict(const Eigen::MatrixXd & transition);
  // 보정: K = P Hᵀ (H P Hᵀ + R)⁻¹, x ← x + K (z − H x), P ← (I − K H) P
  void Correct(const Eigen::VectorXd & measurement, const Eigen::MatrixXd & observation);
  const Eigen::VectorXd & State() const;

private:
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd process_noise_;
  Eigen::MatrixXd measurement_noise_;
};

}  // namespace lekiwi_imu

#endif  // LEKIWI_IMU__KALMAN_FILTER_HPP_
