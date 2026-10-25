#include "lekiwi_lidar/ydlidar_driver.hpp"

#include <stdexcept>

namespace lekiwi_lidar
{

YdlidarDriver::YdlidarDriver(const YdlidarConfig & config)
: config_(config)
{
}

YdlidarDriver::~YdlidarDriver()
{
  // 노드가 어떤 경로로 끝나든 모터 회전을 멈추고 포트를 닫는다.
  Close();
}

template<typename T>
void YdlidarDriver::SetOption(const int property, T value, const char * name)
{
  // SDK는 값의 크기로 타입을 확인하므로 int, bool, float를 정확히 맞춰 넘긴다.
  if (!laser_.setlidaropt(property, &value, sizeof(value))) {
    throw std::runtime_error(std::string("failed to set YDLIDAR option ") + name);
  }
}

void YdlidarDriver::Initialize()
{
  const std::string & port = config_.port;
  if (!laser_.setlidaropt(LidarPropSerialPort, port.c_str(), static_cast<int>(port.size()))) {
    throw std::runtime_error("failed to set YDLIDAR option port");
  }

  // int
  SetOption(LidarPropSerialBaudrate, config_.baudrate, "baudrate");
  SetOption(LidarPropLidarType, config_.lidar_type, "lidar_type");
  SetOption(LidarPropDeviceType, config_.device_type, "device_type");
  SetOption(LidarPropSampleRate, config_.sample_rate, "sample_rate");
  SetOption(LidarPropAbnormalCheckCount, config_.abnormal_check_count, "abnormal_check_count");
  SetOption(LidarPropIntenstiyBit, config_.intensity_bit, "intensity_bit");
  // bool
  SetOption(LidarPropFixedResolution, config_.fixed_resolution, "fixed_resolution");
  SetOption(LidarPropReversion, config_.reversion, "reversion");
  SetOption(LidarPropInverted, config_.inverted, "inverted");
  SetOption(LidarPropAutoReconnect, config_.auto_reconnect, "auto_reconnect");
  SetOption(LidarPropSingleChannel, config_.single_channel, "isSingleChannel");
  SetOption(LidarPropIntenstiy, config_.intensity, "intensity");
  SetOption(LidarPropSupportMotorDtrCtrl, config_.support_motor_dtr, "support_motor_dtr");
  // float (각도 [deg], 거리 [m], 주파수 [Hz])
  SetOption(LidarPropMaxAngle, config_.angle_max_deg, "angle_max");
  SetOption(LidarPropMinAngle, config_.angle_min_deg, "angle_min");
  SetOption(LidarPropMaxRange, config_.range_max, "range_max");
  SetOption(LidarPropMinRange, config_.range_min, "range_min");
  SetOption(LidarPropScanFrequency, config_.frequency_hz, "frequency");

  // initialize()가 중간에 실패해도 Close()가 포트를 닫도록 먼저 표시한다.
  connected_ = true;
  if (!laser_.initialize()) {
    const std::string error = last_error();
    Close();
    throw std::runtime_error("failed to initialize YDLIDAR on " + port + ": " + error);
  }
  if (!laser_.turnOn()) {
    const std::string error = last_error();
    Close();
    throw std::runtime_error("failed to start YDLIDAR scan on " + port + ": " + error);
  }
}

bool YdlidarDriver::Read(LaserScan & scan)
{
  return connected_ && laser_.doProcessSimple(scan);
}

void YdlidarDriver::Close()
{
  if (!connected_) {
    return;
  }
  connected_ = false;
  laser_.turnOff();
  laser_.disconnecting();
}

std::string YdlidarDriver::last_error() const
{
  const char * error = laser_.DescribeError();
  return error != nullptr ? error : "unknown error";
}

}  // namespace lekiwi_lidar
