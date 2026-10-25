#ifndef LEKIWI_SCAN_FILTER__SCAN_FILTER_HPP_
#define LEKIWI_SCAN_FILTER__SCAN_FILTER_HPP_

#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace lekiwi_scan_filter
{

using LaserScan = sensor_msgs::msg::LaserScan;

// 바퀴 가림 구간. base_frame 축 기준 방위각 [rad]
struct WheelSector
{
  double center{0.0};
  double half_width{0.0};
};

// lidar_frame → base_frame 회전 (거꾸로 단 roll 포함 전체 회전)
struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct FilterConfig
{
  double self_radius{0.19};       // [m] 라이다 원점 기준. 이보다 가까운 반사는 로봇 자신
  double min_intensity{0.0};      // 0이면 세기 검사를 하지 않는다
  std::vector<WheelSector> wheel_sectors;  // 비어 있으면 바퀴 구간 가림을 끈다
};

// "center:width" [deg] 목록 → 구간. 0 < width < 360, 유한값이 아니면 std::invalid_argument
std::vector<WheelSector> ParseWheelSectors(const std::vector<std::string> & values);

// self_radius · min_intensity는 유한한 0 이상이어야 한다. 아니면 std::invalid_argument
void ValidateThresholds(double self_radius, double min_intensity);

// 제외한 빔을 NaN으로 바꾼 복사본. 배열 순서 · 각도 · 시간 · frame은 그대로 둔다.
// 제외한 빔을 +inf(빈 공간)로 바꾸지 않는다.
// 제외 조건: 유한값 아님, < max(self_radius, range_min), > range_max,
//           (min_intensity > 0) 세기 부족, (구간 있음) base_frame 방위각이 바퀴 구간 안
// 구간이 있으면 rotation이 필요하다. 설정 · 입력이 맞지 않으면 std::invalid_argument
LaserScan FilterScan(
  const LaserScan & scan, const FilterConfig & config,
  const std::optional<Quaternion> & rotation = std::nullopt);

// 유한한 거리값이 하나라도 있는지
bool HasFiniteRange(const LaserScan & scan);

}  // namespace lekiwi_scan_filter

#endif  // LEKIWI_SCAN_FILTER__SCAN_FILTER_HPP_
