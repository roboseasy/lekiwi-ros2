#ifndef LEKIWI_IMU__IMU_MATH_HPP_
#define LEKIWI_IMU__IMU_MATH_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace lekiwi_imu
{

using Vec3 = std::array<double, 3>;

constexpr double kStandardGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295769;
constexpr double kRadToDeg = 57.295779513082320876798;

inline double DegToRad(const double deg) {return deg * kDegToRad;}
inline double RadToDeg(const double rad) {return rad * kRadToDeg;}

// axis_map: 센서 축을 imu_link 축으로 옮기는 부호 있는 순열 (예: [1, 2, 3], [-2, 1, 3])
// lekiwi_sensors/bmi160_conversion.hpp와 같은 규칙을 사용한다.
inline bool IsValidAxisMap(const std::array<int, 3> & axis_map)
{
  std::array<bool, 3> used{false, false, false};
  for (const int entry : axis_map) {
    const int axis = std::abs(entry);
    if (axis < 1 || axis > 3 || used[static_cast<std::size_t>(axis - 1)]) {
      return false;
    }
    used[static_cast<std::size_t>(axis - 1)] = true;
  }
  return true;
}

inline std::array<int, 3> ParseAxisMap(const std::vector<int64_t> & parameter)
{
  if (parameter.size() != 3U) {
    throw std::invalid_argument("axis_map must contain exactly three integers");
  }
  const std::array<int, 3> axis_map{
    static_cast<int>(parameter[0]),
    static_cast<int>(parameter[1]),
    static_cast<int>(parameter[2])};
  if (!IsValidAxisMap(axis_map)) {
    throw std::invalid_argument("axis_map must be a signed permutation of [1, 2, 3]");
  }
  return axis_map;
}

inline Vec3 ApplyAxisMap(const Vec3 & sensor_vector, const std::array<int, 3> & axis_map)
{
  Vec3 output{};
  for (std::size_t i = 0; i < output.size(); ++i) {
    const int entry = axis_map[i];
    const auto source = static_cast<std::size_t>(std::abs(entry) - 1);
    output[i] = (entry < 0 ? -1.0 : 1.0) * sensor_vector[source];
  }
  return output;
}

inline double Norm(const Vec3 & v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace lekiwi_imu

#endif  // LEKIWI_IMU__IMU_MATH_HPP_
