#ifndef LEKIWI_MOTOR_DRIVER__FEETECH_BUS_HPP_
#define LEKIWI_MOTOR_DRIVER__FEETECH_BUS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lekiwi_motor_driver
{

// Feetech 프로토콜 0 패킷. 시리얼 없이 시험할 수 있도록 함수로 둔다.
namespace feetech
{

constexpr uint8_t kHeader = 0xFF;
constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kInstructionRead = 0x02;
constexpr uint8_t kInstructionWrite = 0x03;
constexpr uint8_t kInstructionSyncWrite = 0x83;

// 체크섬 = ~(ID부터 마지막 파라미터까지의 합)
uint8_t Checksum(const uint8_t * bytes, std::size_t length);

// FF FF ID 04 02 ADDRESS LENGTH CHECKSUM
std::vector<uint8_t> BuildReadPacket(uint8_t id, uint8_t address, uint8_t length);

// FF FF ID LEN 03 ADDRESS DATA... CHECKSUM. 응답(상태 패킷)이 온다.
std::vector<uint8_t> BuildWritePacket(uint8_t id, uint8_t address, const std::vector<uint8_t> & data);

// FF FF FE LEN 83 ADDRESS DATA_LENGTH [ID DATA...]... CHECKSUM. 응답이 없다.
// entries의 DATA 길이는 모두 data_length와 같아야 한다.
std::vector<uint8_t> BuildSyncWritePacket(
  uint8_t address, uint8_t data_length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & entries);

}  // namespace feetech

// Feetech STS 시리얼 버스(프로토콜 0) 드라이버.
// Read는 모든 모터에 쓰지만, Write · SyncWrite는 노드가 쓰기를 연 바퀴 · 팔 ID에만 보낸다.
class FeetechBus
{
public:
  FeetechBus(const std::string & port, int baud_rate, int read_timeout_ms);
  ~FeetechBus();
  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

  // 포트를 열고 1 Mbps raw 모드로 설정한다. 실패하면 std::runtime_error를 던진다.
  void Open();
  void Close();

  // id 모터의 address부터 length 바이트를 읽는다.
  // 응답이 없거나 체크섬이 틀리면 false. error에는 응답 패킷의 상태 바이트가 들어간다.
  bool Read(uint8_t id, uint8_t address, uint8_t length, std::vector<uint8_t> & data, uint8_t & error);

  // id 모터의 address부터 data를 쓰고 응답(ACK)을 기다린다.
  // 응답이 없거나 체크섬이 틀리면 false. error에는 응답 패킷의 상태 바이트가 들어간다.
  bool Write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data, uint8_t & error);

  // 여러 모터의 같은 레지스터에 한 패킷으로 쓴다. 모터는 응답하지 않으므로 보낸 것만 확인한다.
  bool SyncWrite(
    uint8_t address, uint8_t data_length,
    const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & entries);

  // 다른 프로그램이 이 포트를 새로 열지 못하게 막았는지 (TIOCEXCL)
  bool exclusive() const {return exclusive_;}

private:
  bool WriteAll(const uint8_t * bytes, std::size_t size);
  bool ReceiveStatus(uint8_t id, uint8_t length, std::vector<uint8_t> & data, uint8_t & error);

  std::string port_;
  int baud_rate_{1000000};
  int read_timeout_ms_{10};
  int fd_{-1};
  bool exclusive_{false};
};

}  // namespace lekiwi_motor_driver

#endif  // LEKIWI_MOTOR_DRIVER__FEETECH_BUS_HPP_
