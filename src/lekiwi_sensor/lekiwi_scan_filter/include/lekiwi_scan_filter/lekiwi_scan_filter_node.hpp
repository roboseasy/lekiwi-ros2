#ifndef LEKIWI_SCAN_FILTER__LEKIWI_SCAN_FILTER_NODE_HPP_
#define LEKIWI_SCAN_FILTER__LEKIWI_SCAN_FILTER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lekiwi_scan_filter/scan_filter.hpp"

namespace lekiwi_scan_filter
{

// 라이다 스캔에서 로봇 자신의 반사를 NaN으로 지워 다시 퍼블리시한다.
//   input_topic (기본 lekiwi/sensor/scan) → output_topic (기본 lekiwi/sensor/scan_filtered)
class LekiwiScanFilterNode : public rclcpp::Node
{
public:
  LekiwiScanFilterNode();

private:
  // 파라미터 선언·로드
  void DeclareParams();
  void InitializeParams();

  void ScanCallback(const LaserScan::ConstSharedPtr scan);
  // lidar_frame → base_frame 회전. 바퀴 구간 가림이 켜졌을 때만 TF를 조회한다.
  std::optional<Quaternion> LookupRotation(const LaserScan & scan) const;
  void LogFirstScan(const LaserScan & input, const LaserScan & output) const;

  // 설정
  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_;
  bool is_wheel_mask_enabled_{false};
  FilterConfig filter_config_;

  bool is_first_scan_logged_{false};

  // TF (바퀴 구간 가림을 켰을 때만 만든다)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_scan_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
};

}  // namespace lekiwi_scan_filter

#endif  // LEKIWI_SCAN_FILTER__LEKIWI_SCAN_FILTER_NODE_HPP_
