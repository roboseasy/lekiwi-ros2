#ifndef LEKIWI_CAMERA__V4L2_CAPTURE_HPP_
#define LEKIWI_CAMERA__V4L2_CAPTURE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lekiwi_camera
{

struct V4l2Config
{
  std::string device{"/dev/video0"};
  int width{640};
  int height{480};
  int fps{30};
  int buffer_count{4};
};

struct V4l2Frame
{
  std::vector<uint8_t> jpeg;                          // 카메라가 보낸 MJPEG 프레임 그대로
  bool has_capture_time{false};
  std::chrono::steady_clock::time_point capture_time; // 드라이버가 기록한 촬영 시각 (CLOCK_MONOTONIC)
};

// Linux V4L2로 USB 카메라를 MJPEG 형식으로 열고 mmap 버퍼에서 프레임을 꺼내는 드라이버
class V4l2Capture
{
public:
  explicit V4l2Capture(const V4l2Config & config);
  ~V4l2Capture();
  V4l2Capture(const V4l2Capture &) = delete;
  V4l2Capture & operator=(const V4l2Capture &) = delete;

  // 장치 열기 → 영상 장치 확인 → MJPEG 크기 · fps 설정 → 버퍼 준비 → 촬영 시작
  // 실패하면 장치를 닫고 std::runtime_error를 던진다. 다시 열 때도 그대로 쓴다.
  void Open();
  // 촬영을 멈추고 장치를 닫는다. 여러 번 불러도 된다.
  void Close();
  // 새 프레임을 timeout_ms까지 기다린다. 프레임이 여러 개 쌓였으면 가장 최근 것만 쓴다.
  bool Read(V4l2Frame & frame, int timeout_ms);

  bool is_open() const {return fd_ >= 0;}
  double actual_fps() const {return actual_fps_;}

private:
  struct Buffer
  {
    void * start{nullptr};
    std::size_t length{0};
  };

  int Ioctl(unsigned long request, void * arg) const;

  V4l2Config config_;
  int fd_{-1};
  bool streaming_{false};
  std::vector<Buffer> buffers_;
  double actual_fps_{0.0};
};

}  // namespace lekiwi_camera

#endif  // LEKIWI_CAMERA__V4L2_CAPTURE_HPP_
