#include "lekiwi_lidar/lekiwi_lidar_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lekiwi_lidar
{

LekiwiLidarNode::LekiwiLidarNode()
: Node("lekiwi_lidar_node")
{
  // 필요한 파라미터 선언 (기본값 설정) → 로드·검증
  DeclareParams();
  InitializeParams();

  // Tmini Plus 연결과 회전 시작. 실패하면 예외가 main까지 올라가 노드를 종료한다.
  driver_ = std::make_unique<YdlidarDriver>(driver_config_);
  driver_->Initialize();

  pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());

  // Read()가 한 바퀴 스캔이 모일 때까지 기다리므로 실제 발행 주기는 라이다 회전 속도를 따른다.
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {TimerCallback();});

  RCLCPP_INFO(
    get_logger(),
    "YDLIDAR ready on %s: %d baud, %.1f Hz, angle %.0f~%.0f deg, range %.2f~%.1f m, frame=%s",
    driver_config_.port.c_str(), driver_config_.baudrate, driver_config_.frequency_hz,
    driver_config_.angle_min_deg, driver_config_.angle_max_deg,
    driver_config_.range_min, driver_config_.range_max, frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "publishing %s", pub_scan_->get_topic_name());
}

void LekiwiLidarNode::DeclareParams()
{
  // YDLIDAR 연결
  declare_parameter<std::string>("port", "/dev/ttyUSB0");
  declare_parameter<int64_t>("baudrate", 230400);
  declare_parameter<std::string>("frame_id", "lidar_link");
  // Tmini Plus 고정값
  declare_parameter<int64_t>("lidar_type", TYPE_TRIANGLE);
  declare_parameter<int64_t>("device_type", YDLIDAR_TYPE_SERIAL);
  declare_parameter<bool>("isSingleChannel", false);
  declare_parameter<bool>("support_motor_dtr", false);
  declare_parameter<int64_t>("sample_rate", 4);
  declare_parameter<bool>("intensity", true);
  declare_parameter<int64_t>("intensity_bit", 8);
  declare_parameter<int64_t>("abnormal_check_count", 4);
  declare_parameter<bool>("auto_reconnect", true);
  // 스캔 범위
  declare_parameter<double>("frequency", 10.0);
  declare_parameter<double>("angle_min", -180.0);
  declare_parameter<double>("angle_max", 180.0);
  declare_parameter<double>("range_min", 0.05);
  declare_parameter<double>("range_max", 12.0);
  declare_parameter<bool>("fixed_resolution", true);
  // 방향
  declare_parameter<bool>("reversion", false);
  declare_parameter<bool>("inverted", true);
  // 출력
  declare_parameter<std::string>("scan_topic", "lekiwi/sensor/scan");
  declare_parameter<bool>("invalid_range_is_inf", false);
  declare_parameter<bool>("zero_time_increment", true);
}

void LekiwiLidarNode::InitializeParams()
{
  driver_config_.port = get_parameter("port").as_string();
  driver_config_.baudrate = static_cast<int>(get_parameter("baudrate").as_int());
  frame_id_ = get_parameter("frame_id").as_string();

  driver_config_.lidar_type = static_cast<int>(get_parameter("lidar_type").as_int());
  driver_config_.device_type = static_cast<int>(get_parameter("device_type").as_int());
  driver_config_.single_channel = get_parameter("isSingleChannel").as_bool();
  driver_config_.support_motor_dtr = get_parameter("support_motor_dtr").as_bool();
  driver_config_.sample_rate = static_cast<int>(get_parameter("sample_rate").as_int());
  driver_config_.intensity = get_parameter("intensity").as_bool();
  driver_config_.intensity_bit = static_cast<int>(get_parameter("intensity_bit").as_int());
  driver_config_.abnormal_check_count =
    static_cast<int>(get_parameter("abnormal_check_count").as_int());
  driver_config_.auto_reconnect = get_parameter("auto_reconnect").as_bool();

  driver_config_.frequency_hz = static_cast<float>(get_parameter("frequency").as_double());
  driver_config_.angle_min_deg = static_cast<float>(get_parameter("angle_min").as_double());
  driver_config_.angle_max_deg = static_cast<float>(get_parameter("angle_max").as_double());
  driver_config_.range_min = static_cast<float>(get_parameter("range_min").as_double());
  driver_config_.range_max = static_cast<float>(get_parameter("range_max").as_double());
  driver_config_.fixed_resolution = get_parameter("fixed_resolution").as_bool();

  driver_config_.reversion = get_parameter("reversion").as_bool();
  driver_config_.inverted = get_parameter("inverted").as_bool();

  scan_topic_ = get_parameter("scan_topic").as_string();
  invalid_range_is_inf_ = get_parameter("invalid_range_is_inf").as_bool();
  zero_time_increment_ = get_parameter("zero_time_increment").as_bool();

  if (driver_config_.port.empty() || frame_id_.empty() || scan_topic_.empty()) {
    throw std::invalid_argument("port, frame_id and scan_topic must not be empty");
  }
  if (driver_config_.baudrate <= 0 || driver_config_.sample_rate <= 0) {
    throw std::invalid_argument("baudrate and sample_rate must be > 0");
  }
  if (driver_config_.frequency_hz <= 0.0f) {
    throw std::invalid_argument("frequency must be > 0");
  }
  if (driver_config_.angle_min_deg < -180.0f || driver_config_.angle_max_deg > 180.0f ||
    driver_config_.angle_min_deg >= driver_config_.angle_max_deg)
  {
    throw std::invalid_argument("angles must satisfy -180 <= angle_min < angle_max <= 180");
  }
  if (driver_config_.range_min <= 0.0f || driver_config_.range_min >= driver_config_.range_max) {
    throw std::invalid_argument("ranges must satisfy 0 < range_min < range_max");
  }

  if (!zero_time_increment_) {
    RCLCPP_WARN(
      get_logger(),
      "zero_time_increment=false: per-point timing after 'inverted' is not verified");
  }
}

void LekiwiLidarNode::TimerCallback()
{
  ::LaserScan scan;   // SDK 타입
  if (!driver_->Read(scan)) {
    HandleReadFailure();
    return;
  }
  if (consecutive_read_failures_ > 0) {
    RCLCPP_INFO(
      get_logger(), "YDLIDAR scan recovered after %zu failures", consecutive_read_failures_);
    consecutive_read_failures_ = 0;
  }

  const sensor_msgs::msg::LaserScan message = MakeScanMessage(scan);
  pub_scan_->publish(message);

  if (!first_scan_logged_) {
    LogFirstScan(message);
    first_scan_logged_ = true;
  }
}

void LekiwiLidarNode::HandleReadFailure()
{
  ++consecutive_read_failures_;
  // 실패가 이어지면 로그가 쏟아지지 않도록 2초에 한 번만 남긴다.
  // 다시 연결은 auto_reconnect가 켜져 있으면 SDK가 맡는다.
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "YDLIDAR scan failed, consecutive failures=%zu: %s",
    consecutive_read_failures_, driver_->last_error().c_str());
}

void LekiwiLidarNode::LogFirstScan(const sensor_msgs::msg::LaserScan & message) const
{
  std::size_t valid = 0;
  float nearest = std::numeric_limits<float>::infinity();
  for (const float range : message.ranges) {
    if (std::isfinite(range) && range >= message.range_min && range <= message.range_max) {
      ++valid;
      nearest = std::min(nearest, range);
    }
  }
  const double scan_hz = message.scan_time > 0.0f ? 1.0 / message.scan_time : 0.0;
  RCLCPP_INFO(
    get_logger(), "first scan: %zu ranges (%zu valid, nearest %.2f m), %.1f Hz",
    message.ranges.size(), valid, valid > 0 ? nearest : 0.0, scan_hz);
}

sensor_msgs::msg::LaserScan LekiwiLidarNode::MakeScanMessage(const ::LaserScan & scan) const
{
  const LaserConfig & config = scan.config;

  sensor_msgs::msg::LaserScan message;
  // SDK stamp: 첫 측정점의 시스템 시각(CLOCK_REALTIME) [ns]
  message.header.stamp = rclcpp::Time(static_cast<int64_t>(scan.stamp), RCL_SYSTEM_TIME);
  message.header.frame_id = frame_id_;
  message.angle_min = config.min_angle;
  message.angle_max = config.max_angle;
  message.angle_increment = config.angle_increment;
  message.scan_time = config.scan_time;
  message.time_increment = zero_time_increment_ ? 0.0f : config.time_increment;
  message.range_min = config.min_range;
  message.range_max = config.max_range;

  if (config.angle_increment <= 0.0f) {
    return message;
  }
  const int size =
    static_cast<int>((config.max_angle - config.min_angle) / config.angle_increment) + 1;
  if (size <= 0) {
    return message;
  }

  // 측정점이 없는 칸은 0.0(무효) 또는 +inf로 채운다.
  const float empty_range =
    invalid_range_is_inf_ ? std::numeric_limits<float>::infinity() : 0.0f;
  message.ranges.assign(static_cast<std::size_t>(size), empty_range);
  message.intensities.assign(static_cast<std::size_t>(size), 0.0f);

  // ydlidar_ros2_driver와 같은 규칙으로 각 측정점을 각도에 맞는 칸에 넣는다.
  for (const auto & point : scan.points) {
    const int index =
      static_cast<int>(std::ceil((point.angle - config.min_angle) / config.angle_increment));
    if (index < 0 || index >= size || point.range < config.min_range) {
      continue;
    }
    message.ranges[static_cast<std::size_t>(index)] = point.range;
    message.intensities[static_cast<std::size_t>(index)] = point.intensity;
  }
  return message;
}

}  // namespace lekiwi_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<lekiwi_lidar::LekiwiLidarNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lekiwi_lidar_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
