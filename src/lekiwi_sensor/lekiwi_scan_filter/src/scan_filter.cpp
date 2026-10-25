#include "lekiwi_scan_filter/scan_filter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace lekiwi_scan_filter
{

namespace
{

constexpr double DEG_TO_RAD = M_PI / 180.0;

std::string Trim(const std::string & text)
{
  const auto is_space = [](const unsigned char character) {return std::isspace(character) != 0;};
  const auto begin = std::find_if_not(text.begin(), text.end(), is_space);
  const auto end = std::find_if_not(text.rbegin(), text.rend(), is_space).base();
  return begin < end ? std::string(begin, end) : std::string();
}

// 숫자 하나만 있어야 한다 (앞뒤 공백 허용). nan · inf는 여기서 통과하고 호출한 쪽에서 거른다.
bool ParseNumber(const std::string & text, double & number)
{
  const std::string trimmed = Trim(text);
  if (trimmed.empty()) {
    return false;
  }
  char * end = nullptr;
  number = std::strtod(trimmed.c_str(), &end);
  return end == trimmed.c_str() + trimmed.size();
}

Quaternion Normalize(const Quaternion & rotation)
{
  const double norm = std::sqrt(
    rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w);
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("lidar-to-base rotation is not a valid quaternion");
  }
  return Quaternion{rotation.x / norm, rotation.y / norm, rotation.z / norm, rotation.w / norm};
}

// 라이다 빔 방향 (cos, sin, 0)을 base_frame으로 돌린 뒤의 방위각
double BaseBearing(const double angle, const Quaternion & q)
{
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  const double bx = (1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * ca + 2.0 * (q.x * q.y - q.z * q.w) * sa;
  const double by = 2.0 * (q.x * q.y + q.z * q.w) * ca + (1.0 - 2.0 * (q.x * q.x + q.z * q.z)) * sa;
  return std::atan2(by, bx);
}

bool IsInWheelSector(const double bearing, const std::vector<WheelSector> & sectors)
{
  return std::any_of(
    sectors.begin(), sectors.end(), [bearing](const WheelSector & sector) {
      const double difference = bearing - sector.center;
      return std::abs(std::atan2(std::sin(difference), std::cos(difference))) <= sector.half_width;
    });
}

}  // namespace

std::vector<WheelSector> ParseWheelSectors(const std::vector<std::string> & values)
{
  std::vector<WheelSector> sectors;
  sectors.reserve(values.size());
  for (const std::string & value : values) {
    const std::size_t colon = value.find(':');
    double center = 0.0;
    double width = 0.0;
    const bool is_parsed =
      colon != std::string::npos && value.find(':', colon + 1) == std::string::npos &&
      ParseNumber(value.substr(0, colon), center) && ParseNumber(value.substr(colon + 1), width);
    if (!is_parsed || !std::isfinite(center) || !std::isfinite(width) ||
      width <= 0.0 || width >= 360.0)
    {
      throw std::invalid_argument(
              "wheel sector must be finite center:width degrees, 0 < width < 360: '" + value + "'");
    }
    sectors.push_back(WheelSector{center * DEG_TO_RAD, width * DEG_TO_RAD / 2.0});
  }
  return sectors;
}

void ValidateThresholds(const double self_radius, const double min_intensity)
{
  if (!std::isfinite(self_radius) || self_radius < 0.0 ||
    !std::isfinite(min_intensity) || min_intensity < 0.0)
  {
    throw std::invalid_argument("radius and intensity threshold must be finite and nonnegative");
  }
}

LaserScan FilterScan(
  const LaserScan & scan, const FilterConfig & config,
  const std::optional<Quaternion> & rotation)
{
  ValidateThresholds(config.self_radius, config.min_intensity);
  const bool has_intensity_threshold = config.min_intensity > 0.0;
  if (has_intensity_threshold && scan.intensities.size() != scan.ranges.size()) {
    throw std::invalid_argument("intensity filtering requested but intensities are missing");
  }
  const bool has_wheel_sectors = !config.wheel_sectors.empty();
  if (has_wheel_sectors && !rotation.has_value()) {
    throw std::invalid_argument("wheel masking requires lidar-to-base rotation");
  }
  const Quaternion lidar_to_base = has_wheel_sectors ? Normalize(*rotation) : Quaternion{};

  const double min_range = std::max(config.self_radius, static_cast<double>(scan.range_min));
  const double max_range = scan.range_max;
  const float rejected = std::numeric_limits<float>::quiet_NaN();

  LaserScan filtered = scan;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double distance = scan.ranges[i];
    const bool is_out_of_range = distance < min_range || max_range < distance;
    bool is_rejected = !std::isfinite(distance) || is_out_of_range;
    if (!is_rejected && has_intensity_threshold) {
      const double intensity = scan.intensities[i];
      is_rejected = !std::isfinite(intensity) || intensity < config.min_intensity;
    }
    if (!is_rejected && has_wheel_sectors) {
      const double angle =
        static_cast<double>(scan.angle_min) + static_cast<double>(i) * scan.angle_increment;
      is_rejected = IsInWheelSector(BaseBearing(angle, lidar_to_base), config.wheel_sectors);
    }
    if (is_rejected) {
      filtered.ranges[i] = rejected;
    }
  }
  return filtered;
}

bool HasFiniteRange(const LaserScan & scan)
{
  return std::any_of(
    scan.ranges.begin(), scan.ranges.end(), [](const float range) {return std::isfinite(range);});
}

}  // namespace lekiwi_scan_filter
