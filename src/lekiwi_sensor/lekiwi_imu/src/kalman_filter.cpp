#include "lekiwi_imu/kalman_filter.hpp"

namespace lekiwi_imu
{

void KalmanFilter::Initialize(
  const int state_size, const int measurement_size, const double process_noise,
  const double measurement_noise)
{
  state_ = Eigen::VectorXd::Zero(state_size);
  covariance_ = Eigen::MatrixXd::Identity(state_size, state_size);
  process_noise_ = Eigen::MatrixXd::Identity(state_size, state_size) * process_noise;
  measurement_noise_ = Eigen::MatrixXd::Identity(measurement_size, measurement_size) * measurement_noise;
}

void KalmanFilter::SetState(const Eigen::VectorXd & state)
{
  state_ = state;
}

void KalmanFilter::Predict(const Eigen::MatrixXd & transition)
{
  state_ = transition * state_;
  covariance_ = transition * covariance_ * transition.transpose() + process_noise_;
}

void KalmanFilter::Correct(const Eigen::VectorXd & measurement, const Eigen::MatrixXd & observation)
{
  const Eigen::MatrixXd innovation_covariance =
    observation * covariance_ * observation.transpose() + measurement_noise_;
  // K = P Hᵀ S⁻¹. S가 대칭이므로 S Kᵀ = H Pᵀ를 풀어 역행렬을 만들지 않는다.
  const Eigen::MatrixXd gain =
    innovation_covariance.ldlt().solve(observation * covariance_.transpose()).transpose();
  state_ += gain * (measurement - observation * state_);
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(covariance_.rows(), covariance_.cols());
  covariance_ = (identity - gain * observation) * covariance_;
}

const Eigen::VectorXd & KalmanFilter::State() const
{
  return state_;
}

}  // namespace lekiwi_imu
