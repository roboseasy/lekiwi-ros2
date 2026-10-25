// Feetech 패킷과 STS3215 값 변환 시험. 시리얼 · 하드웨어를 쓰지 않는다.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "lekiwi_motor_driver/feetech_bus.hpp"
#include "lekiwi_motor_driver/sts3215.hpp"

namespace
{

using lekiwi_motor_driver::feetech::BuildReadPacket;
using lekiwi_motor_driver::feetech::BuildSyncWritePacket;
using lekiwi_motor_driver::feetech::BuildWritePacket;
namespace sts3215 = lekiwi_motor_driver::sts3215;

// ID부터 체크섬까지 더한 값의 하위 바이트가 0xFF면 체크섬이 맞다.
bool HasValidChecksum(const std::vector<uint8_t> & packet)
{
  unsigned int sum = 0;
  for (std::size_t i = 2; i < packet.size(); ++i) {
    sum += packet[i];
  }
  return (sum & 0xFF) == 0xFF;
}

TEST(FeetechProtocol, ReadPacketMatchesProtocol)
{
  const std::vector<uint8_t> expected{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
  EXPECT_EQ(BuildReadPacket(1, 56, 2), expected);
}

TEST(FeetechProtocol, WritePacketMatchesProtocol)
{
  // 바퀴 8 토크 켜기: FF FF 08 04 03 28 01 C7
  const std::vector<uint8_t> expected{0xFF, 0xFF, 0x08, 0x04, 0x03, 0x28, 0x01, 0xC7};
  EXPECT_EQ(BuildWritePacket(8, sts3215::kAddrTorqueEnable, {1}), expected);
}

TEST(FeetechProtocol, SyncWritePacketLayout)
{
  const auto packet = BuildSyncWritePacket(
    sts3215::kAddrGoalSpeed, 2,
    {{7, {0x00, 0x00}}, {8, {0x8C, 0x02}}, {9, {0x8C, 0x82}}});
  ASSERT_EQ(packet.size(), 17u);
  EXPECT_EQ(packet[2], 0xFE);   // 브로드캐스트
  EXPECT_EQ(packet[3], 13);     // (2 + 1) × 3 + 4
  EXPECT_EQ(packet[4], 0x83);
  EXPECT_EQ(packet[5], 46);
  EXPECT_EQ(packet[6], 2);
  EXPECT_EQ(packet[7], 7);
  EXPECT_EQ(packet[10], 8);
  EXPECT_EQ(packet[13], 9);
  EXPECT_TRUE(HasValidChecksum(packet));
}

TEST(FeetechProtocol, SyncWriteRejectsBadInput)
{
  EXPECT_THROW(BuildSyncWritePacket(46, 2, {{7, {0x00}}}), std::invalid_argument);
  EXPECT_THROW(BuildSyncWritePacket(46, 2, {}), std::invalid_argument);
}

TEST(Sts3215, SignMagnitudeRoundTrip)
{
  for (const int value : {0, 1, 652, 1500, -1, -652, -1500, 32767, -32767}) {
    EXPECT_EQ(sts3215::DecodeSignMagnitude(sts3215::EncodeSignMagnitude(value, 15), 15), value);
  }
  EXPECT_EQ(sts3215::EncodeSignMagnitude(-100, 15), 0x8064);
  EXPECT_EQ(sts3215::EncodeSignMagnitude(40000, 15), 0x7FFF);    // 크기를 자른다
  EXPECT_EQ(sts3215::EncodeSignMagnitude(-40000, 15), 0xFFFF);
  // Homing_Offset은 부호 비트 11이다 (LeRobot ENCODINGS_TABLE). 보정 JSON의 homing_offset과 같은 값
  EXPECT_EQ(sts3215::DecodeSignMagnitude(0x0800 | 1234, 11), -1234);
  EXPECT_EQ(sts3215::DecodeSignMagnitude(1234, 11), 1234);
  EXPECT_EQ(sts3215::kEepromBlockLength, 24);   // 9 Min_Position_Limit ~ 32 Homing_Offset
}

TEST(Sts3215, SpeedToTicksScalesAndClamps)
{
  const double scale = 4096.0 / (2.0 * M_PI);
  EXPECT_EQ(sts3215::SpeedToTicks(1.0, scale, 1500), 652);
  EXPECT_EQ(sts3215::SpeedToTicks(-1.0, scale, 1500), -652);
  EXPECT_EQ(sts3215::SpeedToTicks(5.0, scale, 1500), 1500);
  EXPECT_EQ(sts3215::SpeedToTicks(-5.0, scale, 1500), -1500);
  EXPECT_EQ(sts3215::SpeedToTicks(1.0, scale, 0), 0);
  EXPECT_EQ(sts3215::SpeedToTicks(std::numeric_limits<double>::quiet_NaN(), scale, 1500), 0);
  EXPECT_EQ(sts3215::SpeedToTicks(std::numeric_limits<double>::infinity(), scale, 1500), 0);
}

TEST(Sts3215, GoalSpeedBytesAreLittleEndianSignMagnitude)
{
  EXPECT_EQ(sts3215::EncodeGoalSpeed(0), (std::vector<uint8_t>{0x00, 0x00}));
  EXPECT_EQ(sts3215::EncodeGoalSpeed(652), (std::vector<uint8_t>{0x8C, 0x02}));
  EXPECT_EQ(sts3215::EncodeGoalSpeed(-652), (std::vector<uint8_t>{0x8C, 0x82}));
}

TEST(Sts3215, GoalPositionBytesAreLittleEndianAndClamped)
{
  EXPECT_EQ(sts3215::EncodeGoalPosition(0), (std::vector<uint8_t>{0x00, 0x00}));
  EXPECT_EQ(sts3215::EncodeGoalPosition(2048), (std::vector<uint8_t>{0x00, 0x08}));
  EXPECT_EQ(sts3215::EncodeGoalPosition(4095), (std::vector<uint8_t>{0xFF, 0x0F}));
  EXPECT_EQ(sts3215::EncodeGoalPosition(5000), (std::vector<uint8_t>{0xFF, 0x0F}));   // 자른다
  EXPECT_EQ(sts3215::EncodeGoalPosition(-20), (std::vector<uint8_t>{0x00, 0x00}));
}

TEST(FeetechProtocol, ArmGoalSyncWritePacketLayout)
{
  // 팔 1 · 6 목표 위치: FF FF FE 0A 83 2A 02 01 00 08 06 FF 0F CHECKSUM
  const auto packet = BuildSyncWritePacket(
    sts3215::kAddrGoalPosition, 2,
    {{1, sts3215::EncodeGoalPosition(2048)}, {6, sts3215::EncodeGoalPosition(4095)}});
  const std::vector<uint8_t> body{
    0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2A, 0x02, 0x01, 0x00, 0x08, 0x06, 0xFF, 0x0F};
  ASSERT_EQ(packet.size(), body.size() + 1);
  EXPECT_EQ(std::vector<uint8_t>(packet.begin(), packet.end() - 1), body);
  EXPECT_TRUE(HasValidChecksum(packet));
}

TEST(Sts3215, PresentBlockDecodes)
{
  // 위치 2048, 속도 -100, 부하 +50 (5.0 %), 12.1 V, 36 °C, 상태 0, 움직임, 전류 10 (65 mA)
  std::vector<uint8_t> block(sts3215::kPresentBlockLength, 0);
  block[0] = 0x00; block[1] = 0x08;
  block[2] = 0x64; block[3] = 0x80;
  block[4] = 0x32; block[5] = 0x00;
  block[6] = 121;
  block[7] = 36;
  block[10] = 1;
  block[13] = 10; block[14] = 0;
  const auto state = sts3215::DecodePresentBlock(block);
  EXPECT_EQ(state.position_ticks, 2048);
  EXPECT_EQ(state.velocity_ticks, -100);
  EXPECT_EQ(state.load_permille, 50);
  EXPECT_NEAR(state.voltage, 12.1, 1e-9);
  EXPECT_NEAR(state.temperature, 36.0, 1e-9);
  EXPECT_TRUE(state.moving);
  EXPECT_NEAR(state.current, 0.065, 1e-9);
}

}  // namespace
