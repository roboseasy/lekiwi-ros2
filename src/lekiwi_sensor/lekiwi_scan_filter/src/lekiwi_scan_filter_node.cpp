#include "lekiwi_scan_filter/lekiwi_scan_filter_node.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <tf2/exceptions.h>

namespace lekiwi_scan_filter
{

LekiwiScanFilterNode::LekiwiScanFilterNode()
: Node("lekiwi_scan_filter_node")
{
  // 필요한 파라미터 선언 (기본값 설정) → 로드·검증
  DeclareParams();
  InitializeParams();

  // 바퀴 구간 가림을 켰을 때만 TF를 구독한다. 끄면 Pi에서 /tf를 받지 않는다.
  if (is_wheel_mask_enabled_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  }

  pub_scan_ = create_publisher<LaserScan>(output_topic_, rclcpp::SensorDataQoS());
  sub_scan_ = create_subscription<LaserScan>(
    input_topic_, rclcpp::SensorDataQoS(),
    [this](const LaserScan::ConstSharedPtr scan) {ScanCallback(scan);});

  const std::string wheel_mask = is_wheel_mask_enabled_ ?
    "on (" + std::to_string(filter_config_.wheel_sectors.size()) + " sectors in " +
    base_frame_ + ")" : "off";
  RCLCPP_INFO(
    get_logger(), "scan filter: %s -> %s, self_radius %.3f m, min_intensity %.1f, wheel mask %s",
    sub_scan_->get_topic_name(), pub_scan_->get_topic_name(), filter_config_.self_radius,
    filter_config_.min_intensity, wheel_mask.c_str());
}

void LekiwiScanFilterNode::DeclareParams()
{
  // 토픽 · 프레임
  declare_parameter<std::string>("input_topic", "lekiwi/sensor/scan");
  declare_parameter<std::string>("output_topic", "lekiwi/sensor/scan_filtered");
  declare_parameter<std::string>("base_frame", "base_footprint");
  // 제외 조건
  declare_parameter<double>("self_radius", 0.19);
  declare_parameter<double>("min_intensity", 0.0);
  declare_parameter<bool>("mask_wheels", false);
  declare_parameter<std::vector<std::string>>(
    "wheel_sectors", std::vector<std::string>{"179.5:38.5", "61.0:35.5", "-59.3:34.0"});
}

void LekiwiScanFilterNode::InitializeParams()
{
  input_topic_ = get_parameter("input_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  filter_config_.self_radius = get_parameter("self_radius").as_double();
  filter_config_.min_intensity = get_parameter("min_intensity").as_double();
  is_wheel_mask_enabled_ = get_parameter("mask_wheels").as_bool();

  if (input_topic_.empty() || output_topic_.empty()) {
    throw std::invalid_argument("input_topic and output_topic must not be empty");
  }
  // 같은 토픽을 다시 구독하면 걸러진 스캔이 다시 들어온다.
  const auto topics = get_node_topics_interface();
  if (topics->resolve_topic_name(input_topic_) == topics->resolve_topic_name(output_topic_)) {
    throw std::invalid_argument("input and output topics must differ");
  }
  ValidateThresholds(filter_config_.self_radius, filter_config_.min_intensity);

  // 구간 목록은 가림을 켰을 때만 해석한다.
  if (is_wheel_mask_enabled_) {
    if (base_frame_.empty()) {
      throw std::invalid_argument("base_frame must not be empty when mask_wheels is true");
    }
    filter_config_.wheel_sectors =
      ParseWheelSectors(get_parameter("wheel_sectors").as_string_array());
  }
}

void LekiwiScanFilterNode::ScanCallback(const LaserScan::ConstSharedPtr scan)
{
  try {
    const LaserScan filtered = FilterScan(*scan, filter_config_, LookupRotation(*scan));
    // 남은 거리값이 없으면 내보내지 않는다. 받는 쪽의 센서 timeout이 동작하게 둔다.
    if (!HasFiniteRange(filtered)) {
      throw std::invalid_argument("no usable ranges after filtering");
    }
    pub_scan_->publish(filtered);
    if (!is_first_scan_logged_) {
      LogFirstScan(*scan, filtered);
      is_first_scan_logged_ = true;
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "scan withheld: %s", error.what());
  } catch (const std::invalid_argument & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "scan withheld: %s", error.what());
  }
}

std::optional<Quaternion> LekiwiScanFilterNode::LookupRotation(const LaserScan & scan) const
{
  if (!is_wheel_mask_enabled_) {
    return std::nullopt;
  }
  // 스캔 시각의 lidar_frame → base_frame. 기다리지 않는다 (없으면 TransformException).
  const auto rotation = tf_buffer_->lookupTransform(
    base_frame_, scan.header.frame_id, rclcpp::Time(scan.header.stamp)).transform.rotation;
  return Quaternion{rotation.x, rotation.y, rotation.z, rotation.w};
}

void LekiwiScanFilterNode::LogFirstScan(const LaserScan & input, const LaserScan & output) const
{
  std::size_t input_valid = 0;
  std::size_t output_valid = 0;
  for (std::size_t i = 0; i < input.ranges.size(); ++i) {
    const bool is_input_valid = std::isfinite(input.ranges[i]) &&
      input.ranges[i] >= input.range_min && input.ranges[i] <= input.range_max;
    input_valid += is_input_valid ? 1U : 0U;
    output_valid += std::isfinite(output.ranges[i]) ? 1U : 0U;
  }
  RCLCPP_INFO(
    get_logger(), "first scan: %zu ranges, %zu valid in, %zu kept (frame %s)",
    output.ranges.size(), input_valid, output_valid, output.header.frame_id.c_str());
}

}  // namespace lekiwi_scan_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<lekiwi_scan_filter::LekiwiScanFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lekiwi_scan_filter_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
