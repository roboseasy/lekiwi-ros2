#ifndef LEKIWI_MOTOR_DRIVER__STS3215_HPP_
#define LEKIWI_MOTOR_DRIVER__STS3215_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

// Feetech STS3215 레지스터 주소와 값 변환.
// 주소·부호 규칙: LeRobot motors/feetech/tables.py (STS_SMS_SERIES_CONTROL_TABLE, ENCODINGS_TABLE)
namespace lekiwi_motor_driver::sts3215
{

constexpr uint16_t kModelNumber = 777;
constexpr int kTicksPerRevolution = 4096;
constexpr int kCenterTicks = 2048;
constexpr int kMaxPositionTicks = kTicksPerRevolution - 1;   // 4095
constexpr double kTwoPi = 6.28318530717958647692;

// 레지스터 주소
constexpr uint8_t kAddrModelNumber = 3;       // 2 bytes
constexpr uint8_t kAddrMinPositionLimit = 9;  // 2 bytes (EEPROM)
constexpr uint8_t kAddrMaxPositionLimit = 11; // 2 bytes (EEPROM)
constexpr uint8_t kAddrPCoefficient = 21;     // 1 byte (EEPROM)
constexpr uint8_t kAddrDCoefficient = 22;     // 1 byte (EEPROM)
constexpr uint8_t kAddrICoefficient = 23;     // 1 byte (EEPROM)
constexpr uint8_t kAddrHomingOffset = 31;     // 2 bytes (EEPROM), 부호-크기 (bit 11)
constexpr uint8_t kAddrOperatingMode = 33;    // 1 byte
constexpr uint8_t kAddrTorqueEnable = 40;     // 1 byte
constexpr uint8_t kAddrGoalPosition = 42;     // 2 bytes
constexpr uint8_t kAddrGoalSpeed = 46;        // 2 bytes, 부호-크기 (bit 15)
constexpr uint8_t kAddrPresentPosition = 56;  // 2 bytes

// Operating_Mode 값
constexpr uint8_t kModePosition = 0;
constexpr uint8_t kModeVelocity = 1;

// Goal_Speed 크기의 최댓값 (부호 비트 15를 뺀 나머지)
constexpr int kMaxSpeedMagnitude = 0x7FFF;

// 한 번에 읽는 구간
constexpr uint8_t kModelNumberLength = 2;
// 9 Min_Position_Limit ~ 32 Homing_Offset 상위 바이트 (팔 EEPROM 설정, 읽기만)
constexpr uint8_t kEepromBlockAddress = kAddrMinPositionLimit;
constexpr uint8_t kEepromBlockLength = kAddrHomingOffset + 2 - kAddrMinPositionLimit;
// 33 Operating_Mode ~ 40 Torque_Enable
constexpr uint8_t kConfigBlockAddress = kAddrOperatingMode;
constexpr uint8_t kConfigBlockLength = kAddrTorqueEnable - kAddrOperatingMode + 1;
// 56 Present_Position ~ 70 Present_Current 상위 바이트
constexpr uint8_t kPresentBlockAddress = kAddrPresentPosition;
constexpr uint8_t kPresentBlockLength = 15;

// Status 레지스터(65) 비트 (scservo_sdk ERRBIT_*)
constexpr uint8_t kStatusVoltage = 1;
constexpr uint8_t kStatusAngleSensor = 2;
constexpr uint8_t kStatusOverheat = 4;
constexpr uint8_t kStatusOvercurrent = 8;
constexpr uint8_t kStatusOverload = 32;
// 하나라도 켜져 있으면 바퀴 토크를 켜지 않는다
constexpr uint8_t kStatusFaultMask =
  kStatusVoltage | kStatusAngleSensor | kStatusOverheat | kStatusOvercurrent | kStatusOverload;

struct PresentState
{
  int position_ticks{0};
  int velocity_ticks{0};    // [tick/s]
  int load_permille{0};     // [0.1 %]
  double voltage{0.0};      // [V]
  double temperature{0.0};  // [degC]
  double current{0.0};      // [A]
  bool moving{false};
  uint8_t status{0};
};

inline uint16_t Word(const uint8_t * bytes)
{
  return static_cast<uint16_t>(bytes[0] | (bytes[1] << 8));   // little endian
}

// Feetech 부호-크기 표현: sign_bit가 1이면 음수, 나머지 비트가 크기
inline int DecodeSignMagnitude(const uint16_t value, const int sign_bit)
{
  const int magnitude = value & ((1 << sign_bit) - 1);
  return (value & (1 << sign_bit)) != 0 ? -magnitude : magnitude;
}

// 56~70 구간 15바이트를 값으로 바꾼다.
inline PresentState DecodePresentBlock(const std::vector<uint8_t> & block)
{
  PresentState state;
  state.position_ticks = DecodeSignMagnitude(Word(&block[0]), 15);  // 56 Present_Position
  state.velocity_ticks = DecodeSignMagnitude(Word(&block[2]), 15);  // 58 Present_Velocity
  state.load_permille = DecodeSignMagnitude(Word(&block[4]), 10);   // 60 Present_Load
  state.voltage = block[6] * 0.1;                                   // 62 Present_Voltage [0.1 V]
  state.temperature = block[7];                                     // 63 Present_Temperature
  state.status = block[9];                                          // 65 Status
  state.moving = block[10] != 0;                                    // 66 Moving
  // 69 Present_Current. Feetech 문서 기준 1 = 6.5 mA
  state.current = DecodeSignMagnitude(Word(&block[13]), 15) * 0.0065;
  return state;
}

inline double TicksToRad(const double ticks)
{
  return ticks * kTwoPi / kTicksPerRevolution;
}

// DecodeSignMagnitude의 반대. 크기는 부호 비트 아래로 잘라낸다.
inline uint16_t EncodeSignMagnitude(const int value, const int sign_bit)
{
  const int magnitude = std::min(std::abs(value), (1 << sign_bit) - 1);
  return static_cast<uint16_t>(value < 0 ? (magnitude | (1 << sign_bit)) : magnitude);
}

// [rad/s] → Goal_Speed [tick/s]. tick_limit으로 크기를 자른다. 유한하지 않으면 0.
inline int SpeedToTicks(const double velocity, const double ticks_per_rad, const int tick_limit)
{
  if (!std::isfinite(velocity)) {
    return 0;
  }
  const double ticks = std::round(velocity * ticks_per_rad);
  const double limit = static_cast<double>(std::min(tick_limit, kMaxSpeedMagnitude));
  return static_cast<int>(std::clamp(ticks, -limit, limit));
}

// Goal_Speed 레지스터 2바이트 (little endian, 부호-크기)
inline std::vector<uint8_t> EncodeGoalSpeed(const int ticks)
{
  const uint16_t word = EncodeSignMagnitude(ticks, 15);
  return {static_cast<uint8_t>(word & 0xFF), static_cast<uint8_t>(word >> 8)};
}

// Goal_Position 레지스터 2바이트 (little endian). 0 ~ 4095로 자른다.
inline std::vector<uint8_t> EncodeGoalPosition(const int ticks)
{
  const int clamped = std::clamp(ticks, 0, kMaxPositionTicks);
  return {static_cast<uint8_t>(clamped & 0xFF), static_cast<uint8_t>(clamped >> 8)};
}

inline std::string StatusToString(const uint8_t status)
{
  std::string text;
  const auto append = [&text](const char * name) {
      if (!text.empty()) {
        text += ", ";
      }
      text += name;
    };
  if (status & kStatusVoltage) {append("voltage");}
  if (status & kStatusAngleSensor) {append("angle sensor");}
  if (status & kStatusOverheat) {append("overheat");}
  if (status & kStatusOvercurrent) {append("overcurrent");}
  if (status & kStatusOverload) {append("overload");}
  return text.empty() ? "unknown" : text;
}

}  // namespace lekiwi_motor_driver::sts3215

#endif  // LEKIWI_MOTOR_DRIVER__STS3215_HPP_
