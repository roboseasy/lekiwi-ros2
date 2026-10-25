#ifndef LEKIWI_IMU__BMI160_DRIVER_HPP_
#define LEKIWI_IMU__BMI160_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "lekiwi_imu/imu_math.hpp"

namespace lekiwi_imu
{

struct Bmi160Config
{
  std::string i2c_device{"/dev/i2c-1"};
  uint8_t i2c_address{0x68};   // SA0=GND: 0x68, SA0=3.3V: 0x69
  int sample_rate_hz{100};     // 25, 50, 100, 200, 400
  int accel_range_g{2};        // 2, 4, 8, 16
  int gyro_range_dps{250};     // 125, 250, 500, 1000, 2000
};

struct Bmi160Sample
{
  Vec3 accel_mps2{};   // 센서 축 기준 가속도 [m/s^2]
  Vec3 gyro_rps{};     // 센서 축 기준 각속도 [rad/s]
};

// Linux i2c-dev로 BMI160 레지스터를 직접 읽고 쓰는 드라이버
class Bmi160Driver
{
public:
  explicit Bmi160Driver(const Bmi160Config & config);
  ~Bmi160Driver();
  Bmi160Driver(const Bmi160Driver &) = delete;
  Bmi160Driver & operator=(const Bmi160Driver &) = delete;

  // 장치 열기 → 칩 ID 확인 → 소프트 리셋 → ODR·범위 설정 → 정상 모드 전환
  // 실패하면 std::runtime_error를 던진다. 재초기화에도 그대로 사용한다.
  void Initialize();
  // 자이로·가속도 12바이트를 한 번에 읽는다. 통신 실패 시 false.
  bool Read(Bmi160Sample & sample);
  void Close();

  uint8_t error_register() const {return error_register_;}

private:
  void Open();
  void WriteRegister(uint8_t reg, uint8_t value);
  bool ReadRegisters(uint8_t reg, uint8_t * data, std::size_t length);
  uint8_t ReadRegister(uint8_t reg);

  Bmi160Config config_;
  int fd_{-1};
  uint8_t odr_code_{0};
  uint8_t accel_range_code_{0};
  uint8_t gyro_range_code_{0};
  double accel_lsb_per_g_{16384.0};
  double gyro_lsb_per_dps_{131.2};
  uint8_t error_register_{0};
};

}  // namespace lekiwi_imu

#endif  // LEKIWI_IMU__BMI160_DRIVER_HPP_
