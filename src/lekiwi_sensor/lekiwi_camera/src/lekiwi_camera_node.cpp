#include "lekiwi_camera/lekiwi_camera_node.hpp"

#include <filesystem>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace lekiwi_camera
{
namespace
{

// by-path 링크가 실제로 가리키는 /dev/videoN을 로그에 보여준다.
std::string ResolveDevice(const std::string & device)
{
  std::error_code error;
  const auto resolved = std::filesystem::canonical(device, error);
  return error ? device : resolved.string();
}

}  // namespace

LekiwiCameraNode::LekiwiCameraNode()
: Node("lekiwi_camera_node")
{
  // 필요한 파라미터 선언 (기본값 설정) → 로드·검증
  DeclareParams();
  InitializeParams();

  // 카메라 열기. 실패하면 예외가 main까지 올라가 노드를 종료한다.
  capture_ = std::make_unique<V4l2Capture>(capture_config_);
  capture_->Open();

  // rqt_image_view · RViz 기본 설정(reliable)으로도 받을 수 있게 reliable QoS를 쓴다.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  pub_image_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    topic_prefix_ + "/image_raw/compressed", qos);
  pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_prefix_ + "/camera_info", qos);

  // Read()가 다음 프레임까지 기다리므로 실제 발행 주기는 카메라 fps를 따른다.
  timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {TimerCallback();});

  RCLCPP_INFO(
    get_logger(), "camera ready on %s (%s): MJPEG %dx%d @ %.1f fps, rotation %d deg, frame=%s",
    capture_config_.device.c_str(), ResolveDevice(capture_config_.device).c_str(),
    capture_config_.width, capture_config_.height, capture_->actual_fps(), rotation_,
    frame_id_.c_str());
  RCLCPP_INFO(
    get_logger(), "publishing %s, %s", pub_image_->get_topic_name(), pub_info_->get_topic_name());
}

void LekiwiCameraNode::DeclareParams()
{
  // 카메라 장치
  declare_parameter<std::string>("device", "/dev/video0");
  declare_parameter<int64_t>("width", 640);
  declare_parameter<int64_t>("height", 480);
  declare_parameter<int64_t>("fps", 30);
  // 출력
  declare_parameter<int64_t>("rotation", 0);
  declare_parameter<int64_t>("jpeg_quality", 80);
  declare_parameter<std::string>("frame_id", "camera_optical_frame");
  declare_parameter<std::string>("topic_prefix", "lekiwi/sensor/camera");
  // 장애 처리
  declare_parameter<int64_t>("frame_timeout_ms", 1000);
  declare_parameter<int64_t>("reopen_after_failures", 3);
}

void LekiwiCameraNode::InitializeParams()
{
  capture_config_.device = get_parameter("device").as_string();
  const int64_t width = get_parameter("width").as_int();
  const int64_t height = get_parameter("height").as_int();
  const int64_t fps = get_parameter("fps").as_int();
  const int64_t rotation = get_parameter("rotation").as_int();
  const int64_t jpeg_quality = get_parameter("jpeg_quality").as_int();
  const int64_t frame_timeout_ms = get_parameter("frame_timeout_ms").as_int();
  frame_id_ = get_parameter("frame_id").as_string();
  topic_prefix_ = get_parameter("topic_prefix").as_string();
  reopen_after_failures_ = get_parameter("reopen_after_failures").as_int();

  if (capture_config_.device.empty() || frame_id_.empty() || topic_prefix_.empty()) {
    throw std::invalid_argument("device, frame_id and topic_prefix must not be empty");
  }
  if (width <= 0 || width > 4096 || height <= 0 || height > 4096) {
    throw std::invalid_argument("width and height must be in (0, 4096]");
  }
  if (fps < 1 || fps > 120) {
    throw std::invalid_argument("fps must be in [1, 120]");
  }
  if (rotation != 0 && rotation != 90 && rotation != 180 && rotation != 270) {
    throw std::invalid_argument("rotation must be 0, 90, 180 or 270");
  }
  if (jpeg_quality < 1 || jpeg_quality > 100) {
    throw std::invalid_argument("jpeg_quality must be in [1, 100]");
  }
  if (frame_timeout_ms < 100 || frame_timeout_ms > 10000) {
    throw std::invalid_argument("frame_timeout_ms must be in [100, 10000]");
  }
  if (reopen_after_failures_ < 1) {
    throw std::invalid_argument("reopen_after_failures must be >= 1");
  }

  capture_config_.width = static_cast<int>(width);
  capture_config_.height = static_cast<int>(height);
  capture_config_.fps = static_cast<int>(fps);
  rotation_ = static_cast<int>(rotation);
  jpeg_quality_ = static_cast<int>(jpeg_quality);
  frame_timeout_ms_ = static_cast<int>(frame_timeout_ms);

  // 90 · 270도 회전이면 가로 · 세로가 바뀐다.
  const bool swap_size = rotation_ == 90 || rotation_ == 270;
  output_width_ = swap_size ? capture_config_.height : capture_config_.width;
  output_height_ = swap_size ? capture_config_.width : capture_config_.height;
}

void LekiwiCameraNode::TimerCallback()
{
  if (!capture_->is_open()) {
    TryReopen();
    return;
  }

  V4l2Frame frame;
  if (!capture_->Read(frame, frame_timeout_ms_)) {
    HandleReadFailure();
    return;
  }
  // MJPEG 프레임은 FF D8(SOI)로 시작한다. 깨진 프레임은 건너뛴다.
  if (frame.jpeg.size() < 4 || frame.jpeg[0] != 0xFF || frame.jpeg[1] != 0xD8) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "skipping a frame that is not JPEG (%zu bytes)",
      frame.jpeg.size());
    return;
  }
  if (consecutive_failures_ > 0) {
    RCLCPP_INFO(
      get_logger(), "camera frames recovered after %zu timeouts", consecutive_failures_);
    consecutive_failures_ = 0;
  }

  const rclcpp::Time stamp = FrameStamp(frame);
  const std::size_t camera_bytes = frame.jpeg.size();

  sensor_msgs::msg::CompressedImage image;
  image.header.stamp = stamp;
  image.header.frame_id = frame_id_;
  image.format = "jpeg";
  if (rotation_ == 0) {
    image.data = std::move(frame.jpeg);   // 카메라 프레임 그대로 (디코딩 없음)
  } else if (!RotateJpeg(frame.jpeg, image.data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "failed to decode a JPEG frame for rotation; skipping");
    return;
  }

  if (!first_frame_logged_) {
    RCLCPP_INFO(
      get_logger(), "first frame: %zu bytes from camera, %zu bytes published, %dx%d",
      camera_bytes, image.data.size(), output_width_, output_height_);
    first_frame_logged_ = true;
  }
  pub_info_->publish(MakeCameraInfo(stamp));
  pub_image_->publish(std::move(image));
}

void LekiwiCameraNode::HandleReadFailure()
{
  ++consecutive_failures_;
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 5000, "no frame from %s within %d ms (consecutive timeouts=%zu)",
    capture_config_.device.c_str(), frame_timeout_ms_, consecutive_failures_);
  // 카메라를 뽑았다 꽂은 경우에 대비해 일정 횟수마다 장치를 다시 연다.
  if (consecutive_failures_ % static_cast<std::size_t>(reopen_after_failures_) == 0) {
    RCLCPP_WARN(get_logger(), "reopening %s", capture_config_.device.c_str());
    capture_->Close();
    last_reopen_attempt_ = {};
  }
}

void LekiwiCameraNode::TryReopen()
{
  // 장치가 없을 때 너무 자주 시도하지 않도록 1초에 한 번만 연다.
  const auto now_time = std::chrono::steady_clock::now();
  if (now_time - last_reopen_attempt_ < std::chrono::seconds(1)) {
    return;
  }
  last_reopen_attempt_ = now_time;
  try {
    capture_->Open();
    RCLCPP_INFO(get_logger(), "reopened %s", capture_config_.device.c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "failed to reopen camera: %s", error.what());
  }
}

rclcpp::Time LekiwiCameraNode::FrameStamp(const V4l2Frame & frame)
{
  // 드라이버가 기록한 촬영 시각(CLOCK_MONOTONIC)만큼 현재 ROS 시각에서 거슬러 올라간다.
  rclcpp::Time stamp = now();
  if (frame.has_capture_time) {
    const auto age = std::chrono::steady_clock::now() - frame.capture_time;
    if (age >= std::chrono::nanoseconds::zero() && age < std::chrono::seconds(1)) {
      stamp -= rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(age));
    }
  }
  return stamp;
}

bool LekiwiCameraNode::RotateJpeg(
  const std::vector<uint8_t> & input, std::vector<uint8_t> & output) const
{
  const cv::Mat image = cv::imdecode(input, cv::IMREAD_COLOR);
  if (image.empty()) {
    return false;
  }
  const int code = rotation_ == 90 ? cv::ROTATE_90_CLOCKWISE :
    rotation_ == 180 ? cv::ROTATE_180 : cv::ROTATE_90_COUNTERCLOCKWISE;
  cv::Mat rotated;
  cv::rotate(image, rotated, code);
  return cv::imencode(".jpg", rotated, output, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
}

sensor_msgs::msg::CameraInfo LekiwiCameraNode::MakeCameraInfo(const rclcpp::Time & stamp) const
{
  // 보정 전: 크기와 frame_id만 채운다. K가 모두 0이면 보정되지 않은 카메라라는 뜻이다.
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame_id_;
  info.width = static_cast<uint32_t>(output_width_);
  info.height = static_cast<uint32_t>(output_height_);
  return info;
}

}  // namespace lekiwi_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<lekiwi_camera::LekiwiCameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lekiwi_camera_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
