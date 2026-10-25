#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "lekiwi_scan_filter/scan_filter.hpp"

namespace lekiwi_scan_filter
{
namespace
{

constexpr double PI = M_PI;

// 원래 Python 테스트와 같은 스캔: 0, 90, 180, 270도 네 빔
LaserScan MakeScan(const std::vector<float> & ranges, const std::vector<float> & intensities = {})
{
  LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.header.stamp.sec = 12;
  scan.header.stamp.nanosec = 345;
  scan.ranges = ranges;
  scan.intensities = intensities;
  scan.range_min = 0.05f;
  scan.range_max = 12.0f;
  scan.angle_min = 0.0f;
  scan.angle_increment = static_cast<float>(PI / 2.0);
  if (!ranges.empty()) {
    scan.angle_max = scan.angle_min + scan.angle_increment * static_cast<float>(ranges.size() - 1);
  }
  scan.time_increment = 0.001f;
  scan.scan_time = 0.16f;
  return scan;
}

FilterConfig MakeConfig(
  const double self_radius, const double min_intensity = 0.0,
  const std::vector<std::string> & sectors = {})
{
  FilterConfig config;
  config.self_radius = self_radius;
  config.min_intensity = min_intensity;
  config.wheel_sectors = ParseWheelSectors(sectors);
  return config;
}

constexpr Quaternion IDENTITY{0.0, 0.0, 0.0, 1.0};
constexpr Quaternion UPSIDE_DOWN{1.0, 0.0, 0.0, 0.0};   // roll = π

}  // namespace

TEST(ScanFilterTest, RejectedBeamsNeverClearObstaclesAndInputIsUnchanged)
{
  const float inf = std::numeric_limits<float>::infinity();
  const LaserScan scan = MakeScan({0.1f, 1.0f, inf, 13.0f});
  const LaserScan out = FilterScan(scan, MakeConfig(0.21));

  EXPECT_FLOAT_EQ(out.ranges[1], 1.0f);
  for (const std::size_t i : {0U, 2U, 3U}) {
    EXPECT_TRUE(std::isnan(out.ranges[i])) << "index " << i;
  }
  // 입력은 바꾸지 않는다.
  EXPECT_FLOAT_EQ(scan.ranges[0], 0.1f);
  EXPECT_TRUE(std::isinf(scan.ranges[2]));
  // 배열 크기 · 시각 · 각도 · 시간 · frame은 그대로
  EXPECT_EQ(out.ranges.size(), scan.ranges.size());
  EXPECT_EQ(out.header, scan.header);
  EXPECT_EQ(out.angle_min, scan.angle_min);
  EXPECT_EQ(out.angle_max, scan.angle_max);
  EXPECT_EQ(out.angle_increment, scan.angle_increment);
  EXPECT_EQ(out.time_increment, scan.time_increment);
  EXPECT_EQ(out.scan_time, scan.scan_time);
  EXPECT_EQ(out.range_min, scan.range_min);
  EXPECT_EQ(out.range_max, scan.range_max);
  EXPECT_EQ(out.intensities, scan.intensities);
}

TEST(ScanFilterTest, RangeMinWinsWhenLargerThanSelfRadius)
{
  LaserScan scan = MakeScan({0.25f, 0.35f});
  scan.range_min = 0.3f;
  const LaserScan out = FilterScan(scan, MakeConfig(0.19));
  EXPECT_TRUE(std::isnan(out.ranges[0]));
  EXPECT_FLOAT_EQ(out.ranges[1], 0.35f);
}

TEST(ScanFilterTest, LidarZeroForEmptyCellIsRejected)
{
  // lekiwi_lidar는 측정점이 없는 칸을 0.0으로 낸다.
  const LaserScan out = FilterScan(MakeScan({0.0f, 2.0f}), MakeConfig(0.19));
  EXPECT_TRUE(std::isnan(out.ranges[0]));
  EXPECT_FLOAT_EQ(out.ranges[1], 2.0f);
}

TEST(ScanFilterTest, UpsideDownMountMasksOppositeSensorBearing)
{
  // 센서 +90도 빔은 거꾸로 단 상태에서 base -90도를 본다.
  const LaserScan out = FilterScan(
    MakeScan({1.0f, 1.0f, 1.0f, 1.0f}), MakeConfig(0.21, 0.0, {"-90:10"}), UPSIDE_DOWN);
  EXPECT_TRUE(std::isnan(out.ranges[1]));
  EXPECT_FLOAT_EQ(out.ranges[0], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[2], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[3], 1.0f);
}

TEST(ScanFilterTest, YawRotationShiftsSectorBearing)
{
  // yaw +90도: 센서 0도 빔 → base +90도
  const double half = PI / 4.0;
  const Quaternion yaw_left{0.0, 0.0, std::sin(half), std::cos(half)};
  const LaserScan out = FilterScan(
    MakeScan({1.0f, 1.0f, 1.0f, 1.0f}), MakeConfig(0.21, 0.0, {"90:10"}), yaw_left);
  EXPECT_TRUE(std::isnan(out.ranges[0]));
  EXPECT_FLOAT_EQ(out.ranges[1], 1.0f);
}

TEST(ScanFilterTest, SectorWrapsAtPi)
{
  const LaserScan out = FilterScan(
    MakeScan({1.0f, 1.0f, 1.0f, 1.0f}), MakeConfig(0.21, 0.0, {"-179:10"}), IDENTITY);
  EXPECT_TRUE(std::isnan(out.ranges[2]));
  EXPECT_FLOAT_EQ(out.ranges[0], 1.0f);
}

TEST(ScanFilterTest, NonUnitQuaternionIsNormalizedAndZeroIsRejected)
{
  const LaserScan scan = MakeScan({1.0f, 1.0f, 1.0f, 1.0f});
  const FilterConfig config = MakeConfig(0.21, 0.0, {"-90:10"});
  const LaserScan out = FilterScan(scan, config, Quaternion{2.0, 0.0, 0.0, 0.0});
  EXPECT_TRUE(std::isnan(out.ranges[1]));
  EXPECT_FLOAT_EQ(out.ranges[3], 1.0f);
  EXPECT_THROW(FilterScan(scan, config, Quaternion{0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}

TEST(ScanFilterTest, IntensityRequiresActualMeasurements)
{
  EXPECT_THROW(FilterScan(MakeScan({1.0f}), MakeConfig(0.21, 25.0)), std::invalid_argument);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const LaserScan out = FilterScan(
    MakeScan({1.0f, 1.0f, 1.0f}, {24.0f, 25.0f, nan}), MakeConfig(0.21, 25.0));
  EXPECT_TRUE(std::isnan(out.ranges[0]));
  EXPECT_FLOAT_EQ(out.ranges[1], 1.0f);
  EXPECT_TRUE(std::isnan(out.ranges[2]));
}

TEST(ScanFilterTest, ZeroIntensityThresholdIgnoresMissingIntensities)
{
  const LaserScan out = FilterScan(MakeScan({1.0f, 2.0f}), MakeConfig(0.21, 0.0));
  EXPECT_FLOAT_EQ(out.ranges[0], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[1], 2.0f);
}

TEST(ScanFilterTest, InvalidConfigurationIsRejected)
{
  for (const std::string value : {"nan:20", "0:360", "0:-1", "0:0", "inf:10", "10", "1:2:3",
      "a:b", "", ":", "10:", " :5"})
  {
    EXPECT_THROW(ParseWheelSectors({value}), std::invalid_argument) << "'" << value << "'";
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(FilterScan(MakeScan({1.0f}), MakeConfig(nan)), std::invalid_argument);
  EXPECT_THROW(FilterScan(MakeScan({1.0f}), MakeConfig(inf)), std::invalid_argument);
  EXPECT_THROW(FilterScan(MakeScan({1.0f}), MakeConfig(-0.1)), std::invalid_argument);
  EXPECT_THROW(FilterScan(MakeScan({1.0f}), MakeConfig(0.21, -1.0)), std::invalid_argument);

  // 구간이 있는데 회전이 없으면 거부
  FilterConfig config = MakeConfig(0.21);
  config.wheel_sectors.push_back(WheelSector{0.0, 0.1});
  EXPECT_THROW(FilterScan(MakeScan({1.0f}), config), std::invalid_argument);
}

TEST(ScanFilterTest, DefaultWheelSectorsParseToRadians)
{
  const std::vector<WheelSector> sectors =
    ParseWheelSectors({"179.5:38.5", "61.0:35.5", " -59.3 : 34.0 "});
  ASSERT_EQ(sectors.size(), 3U);
  EXPECT_NEAR(sectors[0].center, 179.5 * PI / 180.0, 1e-12);
  EXPECT_NEAR(sectors[0].half_width, 19.25 * PI / 180.0, 1e-12);
  EXPECT_NEAR(sectors[1].center, 61.0 * PI / 180.0, 1e-12);
  EXPECT_NEAR(sectors[2].center, -59.3 * PI / 180.0, 1e-12);
  EXPECT_NEAR(sectors[2].half_width, 17.0 * PI / 180.0, 1e-12);
  EXPECT_TRUE(ParseWheelSectors({}).empty());
}

TEST(ScanFilterTest, HasFiniteRangeDetectsFullyMaskedScan)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(HasFiniteRange(MakeScan({nan, inf})));
  EXPECT_FALSE(HasFiniteRange(MakeScan({})));
  EXPECT_TRUE(HasFiniteRange(MakeScan({nan, 1.0f})));
  // 모든 빔이 자기 반사면 결과에 남는 값이 없다.
  EXPECT_FALSE(HasFiniteRange(FilterScan(MakeScan({0.1f, 0.15f}), MakeConfig(0.19))));
}

}  // namespace lekiwi_scan_filter
