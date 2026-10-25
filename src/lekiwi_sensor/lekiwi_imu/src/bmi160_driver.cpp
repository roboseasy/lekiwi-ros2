#include "lekiwi_imu/bmi160_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace lekiwi_imu
{
namespace
{

// BMI160 레지스터 주소 (Bosch BMI160 datasheet)
constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegErr = 0x02;
constexpr uint8_t kRegPmuStatus = 0x03;
constexpr uint8_t kRegData = 0x0C;       // GYR_X_L ~ ACC_Z_H, 12바이트 연속
constexpr uint8_t kRegAccConf = 0x40;
constexpr uint8_t kRegAccRange = 0x41;
constexpr uint8_t kRegGyrConf = 0x42;
constexpr uint8_t kRegGyrRange = 0x43;
constexpr uint8_t kRegCmd = 0x7E;

constexpr uint8_t kChipId = 0xD1;
constexpr uint8_t kCmdSoftReset = 0xB6;
constexpr uint8_t kCmdAccNormal = 0x11;
constexpr uint8_t kCmdGyrNormal = 0x15;
constexpr uint8_t kBandwidthNormal = 0x20;   // acc_bwp / gyr_bwp = normal (bit 5:4 = 0b10)

void SleepMs(const int milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::string Hex(const unsigned value)
{
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "0x%02X", value & 0xFFU);
  return buffer;
}

uint8_t OdrCode(const int hz)
{
  switch (hz) {
    case 25: return 0x06;
    case 50: return 0x07;
    case 100: return 0x08;
    case 200: return 0x09;
    case 400: return 0x0A;
    default:
      throw std::invalid_argument("sample_rate_hz must be one of 25, 50, 100, 200, 400");
  }
}

struct RangeSetting
{
  uint8_t code;
  double lsb;
};

RangeSetting AccelRange(const int g)
{
  switch (g) {
    case 2: return {0x03, 16384.0};
    case 4: return {0x05, 8192.0};
    case 8: return {0x08, 4096.0};
    case 16: return {0x0C, 2048.0};
    default:
      throw std::invalid_argument("accel_range_g must be one of 2, 4, 8, 16");
  }
}

RangeSetting GyroRange(const int dps)
{
  switch (dps) {
    case 125: return {0x04, 262.4};
    case 250: return {0x03, 131.2};
    case 500: return {0x02, 65.6};
    case 1000: return {0x01, 32.8};
    case 2000: return {0x00, 16.4};
    default:
      throw std::invalid_argument("gyro_range_dps must be one of 125, 250, 500, 1000, 2000");
  }
}

int16_t ToInt16(const uint8_t * data)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8));
}

}  // namespace

Bmi160Driver::Bmi160Driver(const Bmi160Config & config)
: config_(config)
{
  // 잘못된 설정은 장치를 열기 전에 알린다.
  odr_code_ = OdrCode(config_.sample_rate_hz);
  const RangeSetting accel = AccelRange(config_.accel_range_g);
  const RangeSetting gyro = GyroRange(config_.gyro_range_dps);
  accel_range_code_ = accel.code;
  accel_lsb_per_g_ = accel.lsb;
  gyro_range_code_ = gyro.code;
  gyro_lsb_per_dps_ = gyro.lsb;
}

Bmi160Driver::~Bmi160Driver()
{
  Close();
}

void Bmi160Driver::Initialize()
{
  Open();

  // 전원 직후에는 첫 응답이 실패할 수 있어 몇 번 다시 읽는다.
  uint8_t chip_id = 0;
  bool responded = false;
  for (int attempt = 0; attempt < 5; ++attempt) {
    if (ReadRegisters(kRegChipId, &chip_id, 1)) {
      responded = true;
      if (chip_id == kChipId) {
        break;
      }
    }
    SleepMs(20);
  }
  if (chip_id != kChipId) {
    throw std::runtime_error(
            "BMI160 not found on " + config_.i2c_device + " address " + Hex(config_.i2c_address) +
            (responded ? " (chip id " + Hex(chip_id) + ", expected 0xD1)" : " (no response)") +
            ". Check wiring, power and the SA0 address.");
  }

  WriteRegister(kRegCmd, kCmdSoftReset);
  SleepMs(100);
  if (!ReadRegisters(kRegChipId, &chip_id, 1) || chip_id != kChipId) {
    throw std::runtime_error("BMI160 did not respond after soft reset");
  }

  WriteRegister(kRegAccConf, static_cast<uint8_t>(kBandwidthNormal | odr_code_));
  WriteRegister(kRegAccRange, accel_range_code_);
  WriteRegister(kRegGyrConf, static_cast<uint8_t>(kBandwidthNormal | odr_code_));
  WriteRegister(kRegGyrRange, gyro_range_code_);

  // suspend → normal 전환 시간: 가속도 3.8ms, 자이로 80ms 이상
  WriteRegister(kRegCmd, kCmdAccNormal);
  SleepMs(10);
  WriteRegister(kRegCmd, kCmdGyrNormal);
  SleepMs(100);

  const uint8_t pmu_status = ReadRegister(kRegPmuStatus);
  const bool accel_normal = ((pmu_status >> 4) & 0x03) == 0x01;
  const bool gyro_normal = ((pmu_status >> 2) & 0x03) == 0x01;
  if (!accel_normal || !gyro_normal) {
    throw std::runtime_error(
            "BMI160 accel/gyro did not enter normal mode (PMU_STATUS=" + Hex(pmu_status) + ")");
  }
  error_register_ = ReadRegister(kRegErr);
}

bool Bmi160Driver::Read(Bmi160Sample & sample)
{
  uint8_t raw[12];
  if (!ReadRegisters(kRegData, raw, sizeof(raw))) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    sample.gyro_rps[axis] = DegToRad(ToInt16(&raw[2 * axis]) / gyro_lsb_per_dps_);
    sample.accel_mps2[axis] = ToInt16(&raw[6 + 2 * axis]) / accel_lsb_per_g_ * kStandardGravity;
  }
  return true;
}

void Bmi160Driver::Close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void Bmi160Driver::Open()
{
  Close();
  fd_ = ::open(config_.i2c_device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error(
            "failed to open " + config_.i2c_device + ": " + std::strerror(errno) +
            " (check that the user is in the i2c or dialout group)");
  }
  if (::ioctl(fd_, I2C_SLAVE, static_cast<int>(config_.i2c_address)) < 0) {
    const std::string message =
      "failed to select I2C address " + Hex(config_.i2c_address) + ": " + std::strerror(errno);
    Close();
    throw std::runtime_error(message);
  }
}

void Bmi160Driver::WriteRegister(const uint8_t reg, const uint8_t value)
{
  const uint8_t packet[2] = {reg, value};
  if (::write(fd_, packet, sizeof(packet)) != static_cast<ssize_t>(sizeof(packet))) {
    throw std::runtime_error(
            "BMI160 register write " + Hex(reg) + " failed: " + std::strerror(errno));
  }
  // suspend 모드에서는 레지스터 쓰기 사이에 450us 이상 간격이 필요하다.
  SleepMs(1);
}

bool Bmi160Driver::ReadRegisters(const uint8_t reg, uint8_t * data, const std::size_t length)
{
  if (fd_ < 0 || data == nullptr || length == 0) {
    return false;
  }
  if (::write(fd_, &reg, 1) != 1) {
    return false;
  }
  return ::read(fd_, data, length) == static_cast<ssize_t>(length);
}

uint8_t Bmi160Driver::ReadRegister(const uint8_t reg)
{
  uint8_t value = 0;
  if (!ReadRegisters(reg, &value, 1)) {
    throw std::runtime_error("BMI160 register read " + Hex(reg) + " failed: " + std::strerror(errno));
  }
  return value;
}

}  // namespace lekiwi_imu
