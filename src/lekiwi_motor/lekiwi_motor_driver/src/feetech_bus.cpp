#include "lekiwi_motor_driver/feetech_bus.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace lekiwi_motor_driver
{
namespace
{

speed_t ToBaudConstant(const int baud_rate)
{
  switch (baud_rate) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 1000000: return B1000000;
    default: return 0;
  }
}

}  // namespace

namespace feetech
{

uint8_t Checksum(const uint8_t * bytes, const std::size_t length)
{
  unsigned int sum = 0;
  for (std::size_t i = 0; i < length; ++i) {
    sum += bytes[i];
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

std::vector<uint8_t> BuildReadPacket(const uint8_t id, const uint8_t address, const uint8_t length)
{
  std::vector<uint8_t> packet{kHeader, kHeader, id, 4, kInstructionRead, address, length, 0};
  packet.back() = Checksum(packet.data() + 2, packet.size() - 3);
  return packet;
}

std::vector<uint8_t> BuildWritePacket(
  const uint8_t id, const uint8_t address, const std::vector<uint8_t> & data)
{
  // LEN = INSTRUCTION + ADDRESS + DATA + CHECKSUM
  std::vector<uint8_t> packet{
    kHeader, kHeader, id, static_cast<uint8_t>(data.size() + 3), kInstructionWrite, address};
  packet.insert(packet.end(), data.begin(), data.end());
  packet.push_back(Checksum(packet.data() + 2, packet.size() - 2));
  return packet;
}

std::vector<uint8_t> BuildSyncWritePacket(
  const uint8_t address, const uint8_t data_length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & entries)
{
  for (const auto & entry : entries) {
    if (entry.second.size() != data_length) {
      throw std::invalid_argument("sync write data length mismatch");
    }
  }
  // LEN = (ID + DATA) × 모터 수 + INSTRUCTION + ADDRESS + DATA_LENGTH + CHECKSUM
  const std::size_t length = (static_cast<std::size_t>(data_length) + 1) * entries.size() + 4;
  if (entries.empty() || length > 0xFF) {
    throw std::invalid_argument("sync write must have 1 or more entries and fit one packet");
  }
  std::vector<uint8_t> packet{
    kHeader, kHeader, kBroadcastId, static_cast<uint8_t>(length), kInstructionSyncWrite, address,
    data_length};
  for (const auto & entry : entries) {
    packet.push_back(entry.first);
    packet.insert(packet.end(), entry.second.begin(), entry.second.end());
  }
  packet.push_back(Checksum(packet.data() + 2, packet.size() - 2));
  return packet;
}

}  // namespace feetech

FeetechBus::FeetechBus(const std::string & port, const int baud_rate, const int read_timeout_ms)
: port_(port), baud_rate_(baud_rate), read_timeout_ms_(read_timeout_ms)
{
}

FeetechBus::~FeetechBus()
{
  Close();
}

void FeetechBus::Open()
{
  Close();
  const speed_t speed = ToBaudConstant(baud_rate_);
  if (speed == 0) {
    throw std::invalid_argument("unsupported baud_rate " + std::to_string(baud_rate_));
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("failed to open " + port_ + ": " + std::strerror(errno));
  }

  termios options{};
  if (tcgetattr(fd_, &options) != 0) {
    const std::string error = std::strerror(errno);
    Close();
    throw std::runtime_error("failed to read serial settings of " + port_ + ": " + error);
  }
  cfmakeraw(&options);
  options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  options.c_cc[VMIN] = 0;    // 기다림은 poll()이 맡는다
  options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
    tcsetattr(fd_, TCSANOW, &options) != 0)
  {
    const std::string error = std::strerror(errno);
    Close();
    throw std::runtime_error("failed to configure " + port_ + ": " + error);
  }

  // 버스는 한 프로그램만 써야 하므로 다른 프로그램(LeRobot 호스트, lekiwi_node 등)이
  // 이 포트를 새로 열지 못하게 한다. 이미 열고 있는 프로그램은 막지 못한다.
  exclusive_ = ::ioctl(fd_, TIOCEXCL) == 0;
  tcflush(fd_, TCIOFLUSH);
}

void FeetechBus::Close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  exclusive_ = false;
}

bool FeetechBus::Read(
  const uint8_t id, const uint8_t address, const uint8_t length,
  std::vector<uint8_t> & data, uint8_t & error)
{
  if (fd_ < 0) {
    return false;
  }
  const std::vector<uint8_t> request = feetech::BuildReadPacket(id, address, length);
  tcflush(fd_, TCIFLUSH);   // 지난 주기에 늦게 도착한 바이트를 버린다
  if (!WriteAll(request.data(), request.size())) {
    return false;
  }
  return ReceiveStatus(id, length, data, error);
}

bool FeetechBus::Write(
  const uint8_t id, const uint8_t address, const std::vector<uint8_t> & data, uint8_t & error)
{
  if (fd_ < 0 || id == feetech::kBroadcastId) {
    return false;   // 브로드캐스트는 응답이 없어 성공을 확인할 수 없다
  }
  const std::vector<uint8_t> request = feetech::BuildWritePacket(id, address, data);
  tcflush(fd_, TCIFLUSH);
  if (!WriteAll(request.data(), request.size())) {
    return false;
  }
  std::vector<uint8_t> unused;
  return ReceiveStatus(id, 0, unused, error);
}

bool FeetechBus::SyncWrite(
  const uint8_t address, const uint8_t data_length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & entries)
{
  if (fd_ < 0) {
    return false;
  }
  const std::vector<uint8_t> request =
    feetech::BuildSyncWritePacket(address, data_length, entries);
  return WriteAll(request.data(), request.size());
}

bool FeetechBus::WriteAll(const uint8_t * bytes, const std::size_t size)
{
  std::size_t written = 0;
  while (written < size) {
    const ssize_t result = ::write(fd_, bytes + written, size - written);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (result == 0) {
      return false;
    }
    written += static_cast<std::size_t>(result);
  }
  return true;
}

bool FeetechBus::ReceiveStatus(
  const uint8_t id, const uint8_t length, std::vector<uint8_t> & data, uint8_t & error)
{
  // 응답: FF FF ID LEN(=length+2) ERROR DATA[length] CHECKSUM
  const std::size_t packet_size = static_cast<std::size_t>(length) + 6;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(read_timeout_ms_);
  std::vector<uint8_t> buffer;
  buffer.reserve(packet_size * 2);
  std::array<uint8_t, 64> chunk{};

  while (true) {
    // 헤더(FF FF ID)를 찾고, 패킷이 다 모였으면 길이와 체크섬을 검사한다.
    for (std::size_t i = 0; i + 3 <= buffer.size(); ++i) {
      if (buffer[i] != feetech::kHeader || buffer[i + 1] != feetech::kHeader ||
        buffer[i + 2] != id)
      {
        continue;
      }
      if (buffer.size() < i + packet_size) {
        break;   // 아직 덜 왔다
      }
      const uint8_t * packet = buffer.data() + i;
      if (packet[3] != length + 2 ||
        packet[packet_size - 1] != feetech::Checksum(packet + 2, packet_size - 3))
      {
        return false;
      }
      error = packet[4];
      data.assign(packet + 5, packet + 5 + length);
      return true;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) {
      return false;
    }
    pollfd poll_fd{fd_, POLLIN, 0};
    const int ready = ::poll(&poll_fd, 1, static_cast<int>((remaining + 999) / 1000));
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (ready == 0 || (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return false;
    }
    const ssize_t count = ::read(fd_, chunk.data(), chunk.size());
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      return false;
    }
    buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + count);
  }
}

}  // namespace lekiwi_motor_driver
