#ifndef LEKIWI_LIDAR__YDLIDAR_DRIVER_HPP_
#define LEKIWI_LIDAR__YDLIDAR_DRIVER_HPP_

#include <string>

#include <CYdLidar.h>

namespace lekiwi_lidar
{

// 기본값은 lekiwi_sensors/param/ydlidar.yaml의 Tmini Plus 설정과 같다.
struct YdlidarConfig
{
  std::string port{"/dev/ttyUSB0"};
  int baudrate{230400};
  int lidar_type{TYPE_TRIANGLE};
  int device_type{YDLIDAR_TYPE_SERIAL};
  int sample_rate{4};            // [K samples/s]
  int intensity_bit{8};
  int abnormal_check_count{4};
  bool single_channel{false};
  bool intensity{true};
  bool fixed_resolution{true};
  bool reversion{false};         // 180도 회전
  bool inverted{true};           // 스캔 각도 부호 반전
  bool auto_reconnect{true};
  bool support_motor_dtr{false};
  float frequency_hz{10.0f};
  float angle_min_deg{-180.0f};
  float angle_max_deg{180.0f};
  float range_min{0.05f};        // [m]
  float range_max{12.0f};        // [m]
};

// YDLidar SDK(CYdLidar)로 Tmini Plus를 열고 한 바퀴씩 스캔을 읽는 드라이버
// 주의: 여기서 쓰는 LaserScan은 SDK의 전역 타입으로, sensor_msgs::msg::LaserScan과 다르다.
class YdlidarDriver
{
public:
  explicit YdlidarDriver(const YdlidarConfig & config);
  ~YdlidarDriver();
  YdlidarDriver(const YdlidarDriver &) = delete;
  YdlidarDriver & operator=(const YdlidarDriver &) = delete;

  // SDK 옵션 설정 → 포트 열기·장치 확인 → 모터 회전과 스캔 시작
  // 실패하면 포트를 닫고 std::runtime_error를 던진다.
  void Initialize();
  // 한 바퀴 스캔이 모일 때까지 기다린다 (SDK 제한 1초). 실패 시 false.
  bool Read(LaserScan & scan);
  // 스캔과 모터를 멈추고 포트를 닫는다. 여러 번 불러도 된다.
  void Close();

  std::string last_error() const;

private:
  template<typename T>
  void SetOption(int property, T value, const char * name);

  YdlidarConfig config_;
  CYdLidar laser_;
  bool connected_{false};
};

}  // namespace lekiwi_lidar

#endif  // LEKIWI_LIDAR__YDLIDAR_DRIVER_HPP_
