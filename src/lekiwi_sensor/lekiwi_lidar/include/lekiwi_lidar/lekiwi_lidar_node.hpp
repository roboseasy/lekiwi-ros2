#ifndef LEKIWI_LIDAR__LEKIWI_LIDAR_NODE_HPP_
#define LEKIWI_LIDAR__LEKIWI_LIDAR_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "lekiwi_lidar/ydlidar_driver.hpp"

namespace lekiwi_lidar
{

// USB의 YDLIDAR Tmini Plus를 YDLidar SDK로 직접 읽어 LaserScan을 퍼블리시한다.
//   scan_topic (기본 lekiwi/sensor/scan) : 라이다 한 바퀴(약 10 Hz)마다 한 번
class LekiwiLidarNode : public rclcpp::Node
{
public:
  LekiwiLidarNode();

private:
  // 파라미터 선언·로드
  void DeclareParams();
  void InitializeParams();

  // 주기 처리
  void TimerCallback();
  void HandleReadFailure();
  void LogFirstScan(const sensor_msgs::msg::LaserScan & message) const;

  // SDK 스캔(측정점 목록) → ROS LaserScan(angle_min부터 각도 순 배열)
  sensor_msgs::msg::LaserScan MakeScanMessage(const ::LaserScan & scan) const;

  // YDLIDAR
  YdlidarConfig driver_config_;
  std::unique_ptr<YdlidarDriver> driver_;
  std::string frame_id_;

  // 출력
  std::string scan_topic_;
  bool invalid_range_is_inf_{false};
  bool zero_time_increment_{true};

  // 읽기 실패 처리
  std::size_t consecutive_read_failures_{0};
  bool first_scan_logged_{false};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lekiwi_lidar

#endif  // LEKIWI_LIDAR__LEKIWI_LIDAR_NODE_HPP_
