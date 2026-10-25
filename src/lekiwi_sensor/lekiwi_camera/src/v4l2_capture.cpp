#include "lekiwi_camera/v4l2_capture.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace lekiwi_camera
{
namespace
{

std::string ErrnoText()
{
  return std::strerror(errno);
}

std::string FourccText(const uint32_t fourcc)
{
  std::string text(4, ' ');
  for (std::size_t i = 0; i < 4; ++i) {
    text[i] = static_cast<char>((fourcc >> (8 * i)) & 0xFF);
  }
  return text;
}

}  // namespace

V4l2Capture::V4l2Capture(const V4l2Config & config)
: config_(config)
{
}

V4l2Capture::~V4l2Capture()
{
  Close();
}

int V4l2Capture::Ioctl(const unsigned long request, void * arg) const
{
  int result = 0;
  do {
    result = ::ioctl(fd_, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

void V4l2Capture::Open()
{
  Close();
  const std::string & device = config_.device;
  fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("failed to open " + device + ": " + ErrnoText());
  }

  try {
    // 1. 촬영 · 스트리밍을 지원하는 영상 장치인지 확인 (USB 카메라는 video-index0만 해당)
    v4l2_capability capability{};
    if (Ioctl(VIDIOC_QUERYCAP, &capability) != 0) {
      throw std::runtime_error("VIDIOC_QUERYCAP failed on " + device + ": " + ErrnoText());
    }
    const uint32_t caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 ?
      capability.device_caps : capability.capabilities;
    if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0 || (caps & V4L2_CAP_STREAMING) == 0) {
      throw std::runtime_error(device + " is not a video capture device (use the video-index0 node)");
    }

    // 2. MJPEG 크기 설정. 드라이버가 다른 값으로 바꾸면 지원하지 않는 설정이다.
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(config_.width);
    format.fmt.pix.height = static_cast<uint32_t>(config_.height);
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (Ioctl(VIDIOC_S_FMT, &format) != 0) {
      throw std::runtime_error("VIDIOC_S_FMT failed on " + device + ": " + ErrnoText());
    }
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG ||
      format.fmt.pix.width != static_cast<uint32_t>(config_.width) ||
      format.fmt.pix.height != static_cast<uint32_t>(config_.height))
    {
      throw std::runtime_error(
              device + " does not support MJPEG " + std::to_string(config_.width) + "x" +
              std::to_string(config_.height) + " (driver chose " +
              FourccText(format.fmt.pix.pixelformat) + " " +
              std::to_string(format.fmt.pix.width) + "x" + std::to_string(format.fmt.pix.height) +
              ")");
    }

    // 3. fps 설정. 드라이버가 정한 실제 값을 기록한다.
    v4l2_streamparm stream_param{};
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_param.parm.capture.timeperframe.numerator = 1;
    stream_param.parm.capture.timeperframe.denominator = static_cast<uint32_t>(config_.fps);
    actual_fps_ = 0.0;
    if (Ioctl(VIDIOC_S_PARM, &stream_param) == 0 &&
      stream_param.parm.capture.timeperframe.numerator > 0)
    {
      actual_fps_ = static_cast<double>(stream_param.parm.capture.timeperframe.denominator) /
        stream_param.parm.capture.timeperframe.numerator;
    }

    // 4. 커널 버퍼를 mmap으로 연결하고 모두 대기열에 넣는다.
    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(config_.buffer_count);
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (Ioctl(VIDIOC_REQBUFS, &request) != 0 || request.count < 2) {
      throw std::runtime_error("VIDIOC_REQBUFS failed on " + device + ": " + ErrnoText());
    }
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (Ioctl(VIDIOC_QUERYBUF, &buffer) != 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF failed on " + device + ": " + ErrnoText());
      }
      void * start = ::mmap(
        nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
      if (start == MAP_FAILED) {
        throw std::runtime_error("mmap failed on " + device + ": " + ErrnoText());
      }
      buffers_.push_back(Buffer{start, buffer.length});
      if (Ioctl(VIDIOC_QBUF, &buffer) != 0) {
        throw std::runtime_error("VIDIOC_QBUF failed on " + device + ": " + ErrnoText());
      }
    }

    // 5. 촬영 시작
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (Ioctl(VIDIOC_STREAMON, &type) != 0) {
      throw std::runtime_error("VIDIOC_STREAMON failed on " + device + ": " + ErrnoText());
    }
    streaming_ = true;
  } catch (...) {
    Close();
    throw;
  }
}

void V4l2Capture::Close()
{
  if (fd_ >= 0 && streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    Ioctl(VIDIOC_STREAMOFF, &type);
  }
  streaming_ = false;
  for (const auto & buffer : buffers_) {
    ::munmap(buffer.start, buffer.length);
  }
  buffers_.clear();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool V4l2Capture::Read(V4l2Frame & frame, const int timeout_ms)
{
  if (!streaming_) {
    return false;
  }
  pollfd poll_fd{fd_, POLLIN, 0};
  const int ready = ::poll(&poll_fd, 1, timeout_ms);
  if (ready <= 0 || (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    return false;
  }

  // 쌓인 프레임을 모두 꺼내 가장 최근 것만 쓰고, 나머지는 바로 돌려준다 (지연 줄이기).
  v4l2_buffer latest{};
  bool have_frame = false;
  while (true) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (Ioctl(VIDIOC_DQBUF, &buffer) != 0) {
      if (errno == EAGAIN) {
        break;
      }
      if (have_frame) {
        Ioctl(VIDIOC_QBUF, &latest);
      }
      return false;
    }
    if (have_frame) {
      Ioctl(VIDIOC_QBUF, &latest);
    }
    latest = buffer;
    have_frame = true;
  }
  if (!have_frame) {
    return false;
  }

  const bool valid = (latest.flags & V4L2_BUF_FLAG_ERROR) == 0 && latest.bytesused > 0 &&
    latest.index < buffers_.size();
  if (valid) {
    const auto * start = static_cast<const uint8_t *>(buffers_[latest.index].start);
    frame.jpeg.assign(start, start + latest.bytesused);
    frame.has_capture_time =
      (latest.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    frame.capture_time = std::chrono::steady_clock::time_point(
      std::chrono::seconds(latest.timestamp.tv_sec) +
      std::chrono::microseconds(latest.timestamp.tv_usec));
  }
  Ioctl(VIDIOC_QBUF, &latest);
  return valid;
}

}  // namespace lekiwi_camera
