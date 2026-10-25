#ifndef LEKIWI_CAMERA__LEKIWI_CAMERA_NODE_HPP_
#define LEKIWI_CAMERA__LEKIWI_CAMERA_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "lekiwi_camera/v4l2_capture.hpp"

namespace lekiwi_camera
{

// USB 카메라 한 대를 V4L2로 직접 읽어 퍼블리시한다. 카메라마다 노드를 하나씩 띄운다.
//   <topic_prefix>/image_raw/compressed : 카메라가 보낸 MJPEG 프레임 (rotation이 0이면 그대로)
//   <topic_prefix>/camera_info          : 크기와 frame_id (보정 전)
class LekiwiCameraNode : public rclcpp::Node
{
public:
  LekiwiCameraNode();

private:
  // 파라미터 선언·로드
  void DeclareParams();
  void InitializeParams();

  // 주기 처리
  void TimerCallback();
  void HandleReadFailure();
  void TryReopen();
  rclcpp::Time FrameStamp(const V4l2Frame & frame);
  bool RotateJpeg(const std::vector<uint8_t> & input, std::vector<uint8_t> & output) const;
  sensor_msgs::msg::CameraInfo MakeCameraInfo(const rclcpp::Time & stamp) const;

  // 카메라
  V4l2Config capture_config_;
  std::unique_ptr<V4l2Capture> capture_;
  std::string frame_id_;
  int rotation_{0};            // 시계 방향 [deg]: 0, 90, 180, 270
  int jpeg_quality_{80};
  int output_width_{640};
  int output_height_{480};

  // 장애 처리
  int frame_timeout_ms_{1000};
  int64_t reopen_after_failures_{3};
  std::size_t consecutive_failures_{0};
  std::chrono::steady_clock::time_point last_reopen_attempt_;
  bool first_frame_logged_{false};

  std::string topic_prefix_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lekiwi_camera

#endif  // LEKIWI_CAMERA__LEKIWI_CAMERA_NODE_HPP_
