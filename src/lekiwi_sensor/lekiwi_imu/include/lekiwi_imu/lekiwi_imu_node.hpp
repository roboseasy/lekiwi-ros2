#ifndef LEKIWI_IMU__LEKIWI_IMU_NODE_HPP_
#define LEKIWI_IMU__LEKIWI_IMU_NODE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#include "lekiwi_imu/bmi160_driver.hpp"
#include "lekiwi_imu/imu_math.hpp"
#include "lekiwi_imu/kalman_filter.hpp"
#include "lekiwi_imu/one_euro_filter.hpp"

namespace lekiwi_imu
{

// Raspberry Pi I2C의 BMI160을 직접 읽어 IMU 토픽을 퍼블리시한다.
//   offset_imu     : 자이로 바이어스(offset)만 제거한 값, 자세 없음
//   lpf_imu        : One Euro 필터 + 자세 추정 (use_low_pass_filter)
//   calibrated_imu : 칼만 필터 + 자세 추정 (use_kalman_filter)
class LekiwiImuNode : public rclcpp::Node
{
public:
  LekiwiImuNode();

private:
  // 파라미터 선언·로드
  void DeclareParams();
  void InitializeParams();
  void InitializeFilters();

  // 주기 처리
  void TimerCallback();
  void HandleReadFailure();
  void CollectGyroBias(const Vec3 & gyro);

  // 필터링 (입력 단위: 가속도 [g], 각속도 [deg/s])
  std::pair<Vec3, Vec3> SmoothWithOneEuro(const Vec3 & accel_g, const Vec3 & gyro_dps, double stamp_sec);
  std::pair<Vec3, Vec3> SmoothWithKalman(const Vec3 & accel_g, const Vec3 & gyro_dps);

  // 자세 추정 (입력 단위: 가속도 [m/s^2], 각속도 [rad/s])
  void InitializeOrientation(const Vec3 & accel);
  void PredictOrientation(double dt, const Vec3 & accel, const Vec3 & gyro);
  bool IsStationary(const tf2::Vector3 & world_accel, const Vec3 & gyro) const;

  // 퍼블리시
  sensor_msgs::msg::Imu MakeImuMessage(
    const rclcpp::Time & stamp, const Vec3 & accel, const Vec3 & gyro,
    bool with_orientation) const;
  void PublishOrientationTf(const rclcpp::Time & stamp);

  // BMI160
  Bmi160Config driver_config_;
  std::unique_ptr<Bmi160Driver> driver_;
  std::array<int, 3> axis_map_{1, 2, 3};
  std::string frame_id_;

  // 자이로 바이어스(offset) 보정
  int64_t gyro_bias_samples_{200};
  double calibration_max_gyro_std_{0.02};
  std::vector<Vec3> bias_buffer_;
  Vec3 gyro_bias_{};
  bool bias_ready_{false};

  // 읽기 실패 처리
  int64_t reinit_after_failures_{100};
  std::size_t consecutive_read_failures_{0};

  // 토픽
  std::string offset_topic_;
  std::string lpf_topic_;
  std::string kf_topic_;

  // 공분산
  double angular_velocity_variance_{0.0004};
  double linear_acceleration_variance_{0.04};
  double orientation_variance_roll_pitch_{0.0025};
  double orientation_variance_yaw_{0.05};

  // 필터
  bool use_low_pass_filter_{true};
  bool use_kalman_filter_{false};
  double accel_min_cutoff_{1.0};
  double accel_beta_{0.1};
  double gyro_min_cutoff_{2.0};
  double gyro_beta_{0.02};
  double dcutoff_{1.0};
  double q_cov_{0.000005};
  double r_cov_{0.0001};
  std::array<OneEuroFilter, 3> accel_one_euro_;
  std::array<OneEuroFilter, 3> gyro_one_euro_;
  KalmanFilter kalman_filter_;
  bool is_kalman_state_set_{false};

  // 자세 추정과 정지 판별
  double zero_vel_accel_threshold_{0.05};
  double zero_vel_gyro_threshold_{1.0};
  double gyro_deadband_{0.01};
  tf2::Quaternion orientation_{0.0, 0.0, 0.0, 1.0};
  bool orientation_ready_{false};
  bool was_stationary_{true};
  double stationary_yaw_{0.0};
  std::chrono::steady_clock::time_point start_time_;
  double prev_time_{0.0};

  // RViz 확인용 자세 TF
  bool publish_orientation_tf_{false};
  std::string orientation_tf_parent_;
  std::string orientation_tf_child_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_offset_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_lpf_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_kf_imu_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lekiwi_imu

#endif  // LEKIWI_IMU__LEKIWI_IMU_NODE_HPP_
