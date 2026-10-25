#include "lekiwi_imu/lekiwi_imu_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lekiwi_imu
{
namespace
{

void SetCovarianceDiagonal(
  std::array<double, 9> & covariance, const double xx, const double yy, const double zz)
{
  covariance.fill(0.0);
  covariance[0] = xx;
  covariance[4] = yy;
  covariance[8] = zz;
}

// 정지 상태의 가속도(= 중력 방향)로 roll/pitch를 구하고 yaw는 주어진 값을 쓴다.
tf2::Quaternion TiltFromGravity(const Vec3 & accel, const double yaw)
{
  const double roll = std::atan2(accel[1], accel[2]);
  const double pitch = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

double YawOf(const tf2::Quaternion & q)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

LekiwiImuNode::LekiwiImuNode()
: Node("lekiwi_imu_node"),
  start_time_(std::chrono::steady_clock::now())
{
  // 필요한 파라미터 선언 (기본값 설정) → 로드·검증 → 필터 초기화
  DeclareParams();
  InitializeParams();
  InitializeFilters();

  // BMI160 초기화. 실패하면 예외가 main까지 올라가 노드를 종료한다.
  driver_ = std::make_unique<Bmi160Driver>(driver_config_);
  driver_->Initialize();
  if (driver_->error_register() != 0) {
    RCLCPP_WARN(get_logger(), "BMI160 ERR_REG=0x%02X after initialization", driver_->error_register());
  }

  const auto qos = rclcpp::SensorDataQoS();
  pub_offset_imu_ = create_publisher<sensor_msgs::msg::Imu>(offset_topic_, qos);
  if (use_low_pass_filter_) {
    pub_lpf_imu_ = create_publisher<sensor_msgs::msg::Imu>(lpf_topic_, qos);
  }
  if (use_kalman_filter_) {
    pub_kf_imu_ = create_publisher<sensor_msgs::msg::Imu>(kf_topic_, qos);
  }
  if (publish_orientation_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  const auto period = std::chrono::duration<double>(1.0 / driver_config_.sample_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {TimerCallback();});

  RCLCPP_INFO(
    get_logger(), "BMI160 ready on %s address 0x%02X: %d Hz, +/-%d g, +/-%d dps, frame=%s",
    driver_config_.i2c_device.c_str(), driver_config_.i2c_address, driver_config_.sample_rate_hz,
    driver_config_.accel_range_g, driver_config_.gyro_range_dps, frame_id_.c_str());
  if (!bias_ready_) {
    RCLCPP_INFO(
      get_logger(), "keep the robot still: collecting %ld gyro bias samples",
      static_cast<long>(gyro_bias_samples_));
  }
}

void LekiwiImuNode::DeclareParams()
{
  // BMI160 연결
  declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  declare_parameter<int64_t>("i2c_address", 0x68);
  declare_parameter<int64_t>("sample_rate_hz", 100);
  declare_parameter<int64_t>("accel_range_g", 2);
  declare_parameter<int64_t>("gyro_range_dps", 250);
  declare_parameter<std::string>("frame_id", "imu_link");
  declare_parameter<std::vector<int64_t>>("axis_map", {1, 2, 3});
  // 자이로 바이어스 보정·장애 처리
  declare_parameter<int64_t>("gyro_bias_samples", 200);
  declare_parameter<double>("calibration_max_gyro_std", 0.02);
  declare_parameter<int64_t>("reinit_after_failures", 100);
  // 토픽
  declare_parameter<std::string>("offset_topic", "lekiwi/sensor/offset_imu");
  declare_parameter<std::string>("lpf_topic", "lekiwi/sensor/lpf_imu");
  declare_parameter<std::string>("kf_topic", "lekiwi/sensor/calibrated_imu");
  // 공분산
  declare_parameter<double>("angular_velocity_variance", 0.0004);
  declare_parameter<double>("linear_acceleration_variance", 0.04);
  declare_parameter<double>("orientation_variance_roll_pitch", 0.0025);
  declare_parameter<double>("orientation_variance_yaw", 0.05);
  // 필터
  declare_parameter<bool>("use_low_pass_filter", true);
  declare_parameter<bool>("use_kalman_filter", false);
  declare_parameter<double>("accel_min_cutoff", 1.0);
  declare_parameter<double>("accel_beta", 0.1);
  declare_parameter<double>("gyro_min_cutoff", 2.0);
  declare_parameter<double>("gyro_beta", 0.02);
  declare_parameter<double>("dcutoff", 1.0);
  declare_parameter<double>("Q_cov", 0.000005);
  declare_parameter<double>("R_cov", 0.0001);
  // 정지 판별
  declare_parameter<double>("zero_vel_accel_threshold", 0.05);
  declare_parameter<double>("zero_vel_gyro_threshold", 1.0);
  declare_parameter<double>("gyro_deadband", 0.01);
  // RViz 확인용 자세 TF
  declare_parameter<bool>("publish_orientation_tf", false);
  declare_parameter<std::string>("orientation_tf_parent", "imu_world");
  declare_parameter<std::string>("orientation_tf_child", "imu_orientation");
}

void LekiwiImuNode::InitializeParams()
{
  driver_config_.i2c_device = get_parameter("i2c_device").as_string();
  const int64_t address = get_parameter("i2c_address").as_int();
  if (address < 0x08 || address > 0x77) {
    throw std::invalid_argument("i2c_address must be a 7-bit I2C address (104=0x68, 105=0x69)");
  }
  driver_config_.i2c_address = static_cast<uint8_t>(address);
  driver_config_.sample_rate_hz = static_cast<int>(get_parameter("sample_rate_hz").as_int());
  driver_config_.accel_range_g = static_cast<int>(get_parameter("accel_range_g").as_int());
  driver_config_.gyro_range_dps = static_cast<int>(get_parameter("gyro_range_dps").as_int());

  frame_id_ = get_parameter("frame_id").as_string();
  axis_map_ = ParseAxisMap(get_parameter("axis_map").as_integer_array());

  gyro_bias_samples_ = get_parameter("gyro_bias_samples").as_int();
  calibration_max_gyro_std_ = get_parameter("calibration_max_gyro_std").as_double();
  reinit_after_failures_ = get_parameter("reinit_after_failures").as_int();

  offset_topic_ = get_parameter("offset_topic").as_string();
  lpf_topic_ = get_parameter("lpf_topic").as_string();
  kf_topic_ = get_parameter("kf_topic").as_string();

  angular_velocity_variance_ = get_parameter("angular_velocity_variance").as_double();
  linear_acceleration_variance_ = get_parameter("linear_acceleration_variance").as_double();
  orientation_variance_roll_pitch_ = get_parameter("orientation_variance_roll_pitch").as_double();
  orientation_variance_yaw_ = get_parameter("orientation_variance_yaw").as_double();

  use_low_pass_filter_ = get_parameter("use_low_pass_filter").as_bool();
  use_kalman_filter_ = get_parameter("use_kalman_filter").as_bool();
  accel_min_cutoff_ = get_parameter("accel_min_cutoff").as_double();
  accel_beta_ = get_parameter("accel_beta").as_double();
  gyro_min_cutoff_ = get_parameter("gyro_min_cutoff").as_double();
  gyro_beta_ = get_parameter("gyro_beta").as_double();
  dcutoff_ = get_parameter("dcutoff").as_double();
  q_cov_ = get_parameter("Q_cov").as_double();
  r_cov_ = get_parameter("R_cov").as_double();

  zero_vel_accel_threshold_ = get_parameter("zero_vel_accel_threshold").as_double();
  zero_vel_gyro_threshold_ = get_parameter("zero_vel_gyro_threshold").as_double();
  gyro_deadband_ = get_parameter("gyro_deadband").as_double();

  publish_orientation_tf_ = get_parameter("publish_orientation_tf").as_bool();
  orientation_tf_parent_ = get_parameter("orientation_tf_parent").as_string();
  orientation_tf_child_ = get_parameter("orientation_tf_child").as_string();

  if (frame_id_.empty() || offset_topic_.empty() || lpf_topic_.empty() || kf_topic_.empty()) {
    throw std::invalid_argument("frame_id and topic names must not be empty");
  }
  if (gyro_bias_samples_ < 0 || gyro_bias_samples_ > 10000) {
    throw std::invalid_argument("gyro_bias_samples must be in [0, 10000]");
  }
  if (calibration_max_gyro_std_ <= 0.0) {
    throw std::invalid_argument("calibration_max_gyro_std must be > 0");
  }
  if (q_cov_ <= 0.0 || r_cov_ <= 0.0) {
    throw std::invalid_argument("Q_cov and R_cov must be > 0");
  }
  if (publish_orientation_tf_ && (orientation_tf_parent_.empty() || orientation_tf_child_.empty())) {
    throw std::invalid_argument("orientation TF frame names must not be empty");
  }

  if (gyro_bias_samples_ == 0) {
    bias_ready_ = true;
    RCLCPP_WARN(get_logger(), "gyro bias calibration is disabled (gyro_bias_samples=0)");
  } else {
    bias_buffer_.reserve(static_cast<std::size_t>(gyro_bias_samples_));
  }
}

void LekiwiImuNode::InitializeFilters()
{
  const double rate_hz = driver_config_.sample_rate_hz;
  for (auto & filter : accel_one_euro_) {
    filter.Configure({rate_hz, accel_min_cutoff_, accel_beta_, dcutoff_});
  }
  for (auto & filter : gyro_one_euro_) {
    filter.Configure({rate_hz, gyro_min_cutoff_, gyro_beta_, dcutoff_});
  }
  kalman_filter_.Initialize(6, 6, q_cov_, r_cov_);   // 6 states: 3 accel + 3 gyro
}

void LekiwiImuNode::TimerCallback()
{
  Bmi160Sample sample;
  if (!driver_->Read(sample)) {
    HandleReadFailure();
    return;
  }
  if (consecutive_read_failures_ > 0) {
    RCLCPP_INFO(
      get_logger(), "BMI160 read recovered after %zu failures", consecutive_read_failures_);
    consecutive_read_failures_ = 0;
  }

  // 센서 축 → imu_link 축
  const Vec3 accel = ApplyAxisMap(sample.accel_mps2, axis_map_);
  Vec3 gyro = ApplyAxisMap(sample.gyro_rps, axis_map_);

  if (!bias_ready_) {
    CollectGyroBias(gyro);
    return;
  }
  for (std::size_t i = 0; i < gyro.size(); ++i) {
    gyro[i] -= gyro_bias_[i];
  }

  const rclcpp::Time stamp = now();
  const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

  // 1. offset IMU: 바이어스만 제거한 값
  pub_offset_imu_->publish(MakeImuMessage(stamp, accel, gyro, false));

  if (!orientation_ready_) {
    InitializeOrientation(accel);
    prev_time_ = t;
    return;
  }
  double dt = t - prev_time_;
  prev_time_ = t;
  if (dt <= 0.0 || dt > 0.5) {
    dt = 1.0 / driver_config_.sample_rate_hz;
  }

  // 2. 필터링. 필터 파라미터는 [g], [deg/s] 단위 기준이라 단위를 바꿔 거른다
  Vec3 accel_g{};
  Vec3 gyro_dps{};
  for (std::size_t i = 0; i < 3; ++i) {
    accel_g[i] = accel[i] / kStandardGravity;
    gyro_dps[i] = RadToDeg(gyro[i]);
  }

  Vec3 accel_in = accel;
  Vec3 gyro_in = gyro;
  Vec3 accel_lpf{};
  Vec3 gyro_lpf{};
  Vec3 accel_kf{};
  Vec3 gyro_kf{};
  if (use_low_pass_filter_) {
    const auto [a, g] = SmoothWithOneEuro(accel_g, gyro_dps, t);
    for (std::size_t i = 0; i < 3; ++i) {
      accel_lpf[i] = a[i] * kStandardGravity;
      gyro_lpf[i] = DegToRad(g[i]);
    }
    accel_in = accel_lpf;
    gyro_in = gyro_lpf;
  }
  if (use_kalman_filter_) {
    const auto [a, g] = SmoothWithKalman(accel_g, gyro_dps);
    for (std::size_t i = 0; i < 3; ++i) {
      accel_kf[i] = a[i] * kStandardGravity;
      gyro_kf[i] = DegToRad(g[i]);
    }
    accel_in = accel_kf;
    gyro_in = gyro_kf;
  }

  // 3. 자세 추정 (칼만 > LPF > offset 순서로 입력 선택)
  PredictOrientation(dt, accel_in, gyro_in);

  if (use_low_pass_filter_) {
    pub_lpf_imu_->publish(MakeImuMessage(stamp, accel_lpf, gyro_lpf, true));
  }
  if (use_kalman_filter_) {
    pub_kf_imu_->publish(MakeImuMessage(stamp, accel_kf, gyro_kf, true));
  }
  if (publish_orientation_tf_) {
    PublishOrientationTf(stamp);
  }
}

void LekiwiImuNode::HandleReadFailure()
{
  ++consecutive_read_failures_;
  if (consecutive_read_failures_ == 1 || consecutive_read_failures_ % 100 == 0) {
    RCLCPP_ERROR(
      get_logger(), "BMI160 read failed, consecutive failures=%zu", consecutive_read_failures_);
  }
  if (reinit_after_failures_ > 0 &&
    consecutive_read_failures_ % static_cast<std::size_t>(reinit_after_failures_) == 0)
  {
    RCLCPP_WARN(get_logger(), "re-initializing BMI160");
    try {
      driver_->Initialize();
      RCLCPP_INFO(get_logger(), "BMI160 re-initialized");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "BMI160 re-initialization failed: %s", error.what());
    }
  }
}

void LekiwiImuNode::CollectGyroBias(const Vec3 & gyro)
{
  bias_buffer_.push_back(gyro);
  if (bias_buffer_.size() < static_cast<std::size_t>(gyro_bias_samples_)) {
    return;
  }

  const auto count = static_cast<double>(bias_buffer_.size());
  Vec3 mean{};
  for (const auto & sample : bias_buffer_) {
    for (std::size_t i = 0; i < 3; ++i) {
      mean[i] += sample[i] / count;
    }
  }
  double max_std = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    double variance = 0.0;
    for (const auto & sample : bias_buffer_) {
      variance += (sample[i] - mean[i]) * (sample[i] - mean[i]) / count;
    }
    max_std = std::max(max_std, std::sqrt(variance));
  }
  bias_buffer_.clear();

  // 수집 중 로봇이 움직였으면 바이어스로 쓰지 않고 다시 수집한다.
  if (max_std > calibration_max_gyro_std_) {
    RCLCPP_WARN(
      get_logger(),
      "robot moved during gyro bias calibration (std %.4f > %.4f rad/s); keep it still, retrying",
      max_std, calibration_max_gyro_std_);
    return;
  }
  gyro_bias_ = mean;
  bias_ready_ = true;
  RCLCPP_INFO(
    get_logger(), "gyro bias ready: [%.5f, %.5f, %.5f] rad/s (std %.5f)",
    gyro_bias_[0], gyro_bias_[1], gyro_bias_[2], max_std);
  std::string topics = pub_offset_imu_->get_topic_name();
  if (pub_lpf_imu_) {
    topics += std::string(", ") + pub_lpf_imu_->get_topic_name();
  }
  if (pub_kf_imu_) {
    topics += std::string(", ") + pub_kf_imu_->get_topic_name();
  }
  RCLCPP_INFO(get_logger(), "publishing %s", topics.c_str());
}

std::pair<Vec3, Vec3> LekiwiImuNode::SmoothWithOneEuro(
  const Vec3 & accel_g, const Vec3 & gyro_dps, const double stamp_sec)
{
  Vec3 accel_lpf{};
  Vec3 gyro_lpf{};
  for (std::size_t i = 0; i < 3; ++i) {
    accel_lpf[i] = accel_one_euro_[i].Filter(accel_g[i], stamp_sec);
    gyro_lpf[i] = gyro_one_euro_[i].Filter(gyro_dps[i], stamp_sec);
  }
  return {accel_lpf, gyro_lpf};
}

std::pair<Vec3, Vec3> LekiwiImuNode::SmoothWithKalman(const Vec3 & accel_g, const Vec3 & gyro_dps)
{
  Eigen::VectorXd measurement(6);
  measurement << accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2];
  if (!is_kalman_state_set_) {
    // 0에서 시작하면 첫 출력이 튀므로 첫 측정값으로 상태를 맞춘다.
    kalman_filter_.SetState(measurement);
    is_kalman_state_set_ = true;
  }
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  kalman_filter_.Predict(identity);
  kalman_filter_.Correct(measurement, identity);
  const Eigen::VectorXd & filtered = kalman_filter_.State();
  return {
    Vec3{filtered(0), filtered(1), filtered(2)},
    Vec3{filtered(3), filtered(4), filtered(5)}};
}

void LekiwiImuNode::InitializeOrientation(const Vec3 & accel)
{
  // 시작 자세: roll/pitch는 중력 방향, yaw는 0
  stationary_yaw_ = 0.0;
  orientation_ = TiltFromGravity(accel, stationary_yaw_);
  was_stationary_ = true;
  orientation_ready_ = true;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(orientation_).getRPY(roll, pitch, yaw);
  RCLCPP_INFO(
    get_logger(), "initial orientation from gravity: roll=%.2f deg, pitch=%.2f deg",
    RadToDeg(roll), RadToDeg(pitch));
}

void LekiwiImuNode::PredictOrientation(const double dt, const Vec3 & accel, const Vec3 & gyro)
{
  // 1) 각속도 적분: 회전축·회전각으로 증분 quaternion을 만든다
  const double omega = Norm(gyro);
  if (omega >= gyro_deadband_) {
    tf2::Quaternion dq;
    dq.setRotation(tf2::Vector3(gyro[0] / omega, gyro[1] / omega, gyro[2] / omega), omega * dt);
    orientation_ = orientation_ * dq;
    orientation_.normalize();
  }

  // 2) 가속도를 world 좌표로 돌린 뒤 중력을 뺀다
  tf2::Vector3 world_accel = tf2::quatRotate(orientation_, tf2::Vector3(accel[0], accel[1], accel[2]));
  world_accel -= tf2::Vector3(0.0, 0.0, kStandardGravity);

  // 3) 정지 판별: 정지 중에는 yaw를 고정하고 roll/pitch를 중력 방향으로 다시 맞춘다
  const bool stationary = IsStationary(world_accel, gyro);
  if (stationary) {
    if (!was_stationary_) {
      stationary_yaw_ = YawOf(orientation_);
    }
    orientation_ = TiltFromGravity(accel, stationary_yaw_);
  }
  was_stationary_ = stationary;
}

bool LekiwiImuNode::IsStationary(const tf2::Vector3 & world_accel, const Vec3 & gyro) const
{
  const double accel_norm_g = world_accel.length() / kStandardGravity;
  const double gyro_norm_dps = RadToDeg(Norm(gyro));
  return accel_norm_g < zero_vel_accel_threshold_ && gyro_norm_dps < zero_vel_gyro_threshold_;
}

sensor_msgs::msg::Imu LekiwiImuNode::MakeImuMessage(
  const rclcpp::Time & stamp, const Vec3 & accel, const Vec3 & gyro,
  const bool with_orientation) const
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id_;
  message.linear_acceleration.x = accel[0];
  message.linear_acceleration.y = accel[1];
  message.linear_acceleration.z = accel[2];
  message.angular_velocity.x = gyro[0];
  message.angular_velocity.y = gyro[1];
  message.angular_velocity.z = gyro[2];
  SetCovarianceDiagonal(
    message.linear_acceleration_covariance, linear_acceleration_variance_,
    linear_acceleration_variance_, linear_acceleration_variance_);
  SetCovarianceDiagonal(
    message.angular_velocity_covariance, angular_velocity_variance_,
    angular_velocity_variance_, angular_velocity_variance_);
  if (with_orientation) {
    message.orientation = tf2::toMsg(orientation_);
    SetCovarianceDiagonal(
      message.orientation_covariance, orientation_variance_roll_pitch_,
      orientation_variance_roll_pitch_, orientation_variance_yaw_);
  } else {
    // REP-145: 자세를 제공하지 않으면 covariance[0] = -1
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
  }
  return message;
}

void LekiwiImuNode::PublishOrientationTf(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = orientation_tf_parent_;
  transform.child_frame_id = orientation_tf_child_;
  transform.transform.rotation = tf2::toMsg(orientation_);
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace lekiwi_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<lekiwi_imu::LekiwiImuNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lekiwi_imu_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
