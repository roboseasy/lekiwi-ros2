#include "lekiwi_base_controller/driver_motor_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>
#include <future>
#include <memory>
#include <stdexcept>
#include <utility>

namespace lekiwi_base_controller
{

WheelStatesFrame ExtractWheelStatesFrame(
  const lekiwi_msgs::msg::MotorStateArray & states,
  const std::array<uint8_t, WHEEL_COUNT> & wheel_ids)
{
  WheelStatesFrame frame;
  frame.stamp_nanoseconds = rclcpp::Time(states.header.stamp).nanoseconds();
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    for (const auto & motor : states.motors) {
      if (motor.id == wheel_ids[i]) {
        frame.is_online[i] = motor.online;
        frame.position_ticks[i] = motor.position_ticks;
        frame.velocity[i] = motor.velocity;
        break;
      }
    }
  }
  return frame;
}

DriverMotorConfig MakeDriverMotorConfig(const BaseControllerParameters & params)
{
  DriverMotorConfig config;
  config.wheel_ids = WheelIdsFromParameter(params.wheel_ids);
  config.max_wheel_speed = params.max_wheel_speed;
  config.feedback.source = params.encoder_feedback_source == "speed" ?
    EncoderFeedbackSource::Speed : EncoderFeedbackSource::Position;
  config.feedback.wheel_directions = WheelDirectionsFromParameter(params.wheel_directions);
  config.feedback.position_ticks_per_revolution =
    static_cast<int>(params.encoder_position_ticks_per_revolution);
  config.feedback.encoder_odom_scale = params.encoder_odom_scale;
  config.feedback.encoder_read_period = params.encoder_read_period;
  config.feedback.encoder_speed_deadband = params.encoder_speed_deadband;
  config.states_topic = params.states_topic;
  config.wheel_command_topic = params.wheel_command_topic;
  config.wheel_torque_service = params.wheel_torque_service;
  config.driver_states_timeout = params.driver_states_timeout;
  config.torque_service_timeout = params.torque_service_timeout;
  config.is_encoder_read_logged = params.is_encoder_read_logged;
  return config;
}

lekiwi_msgs::msg::WheelCommand MakeWheelCommandMessage(
  const WheelCommand & command,
  const std::array<uint8_t, WHEEL_COUNT> & wheel_ids,
  const std::array<double, WHEEL_COUNT> & wheel_directions,
  const rclcpp::Time & stamp)
{
  lekiwi_msgs::msg::WheelCommand msg;
  msg.header.stamp = stamp;
  const auto motor_velocities = ToMotorVelocities(command, wheel_directions);
  msg.ids.assign(wheel_ids.begin(), wheel_ids.end());
  msg.velocity.assign(motor_velocities.begin(), motor_velocities.end());
  return msg;
}

DriverMotorInterface::DriverMotorInterface(rclcpp::Node & node, DriverMotorConfig config)
: BaseMotorInterface(config.max_wheel_speed),
  config_(std::move(config)),
  logger_(node.get_logger().get_child("driver_motor_interface")),
  clock_(node.get_clock()),
  estimator_(config_.feedback)
{
  if (!(config_.driver_states_timeout > 0.0) || !(config_.torque_service_timeout > 0.0)) {
    throw std::invalid_argument("driver_states_timeout and torque_service_timeout must be > 0");
  }
  // 토크는 꺼진 상태로 본다. 켜기는 SetMotorPower(true) → 드라이버 서비스로만 한다
  SetMotorPowerState(false);

  states_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  torque_client_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions states_options;
  states_options.callback_group = states_group_;
  sub_states_ = node.create_subscription<lekiwi_msgs::msg::MotorStateArray>(
    config_.states_topic, rclcpp::SensorDataQoS(),
    [this](const lekiwi_msgs::msg::MotorStateArray::SharedPtr msg) {StatesCallback(msg);},
    states_options);
  // 최신 명령만 의미가 있으므로 깊이 1 (드라이버 구독과 같다)
  pub_wheel_command_ = node.create_publisher<lekiwi_msgs::msg::WheelCommand>(
    config_.wheel_command_topic, rclcpp::QoS(1));
  torque_client_ = node.create_client<std_srvs::srv::SetBool>(
    config_.wheel_torque_service, rclcpp::ServicesQoS(), torque_client_group_);

  RCLCPP_INFO(
    logger_, "using motor driver: states %s, commands %s, torque %s (wheel ids [%u, %u, %u])",
    sub_states_->get_topic_name(), pub_wheel_command_->get_topic_name(),
    torque_client_->get_service_name(),
    config_.wheel_ids[0], config_.wheel_ids[1], config_.wheel_ids[2]);
}

void DriverMotorInterface::Connect()
{
  // 버스는 드라이버가 연다. 연결 여부는 states로만 판단한다
}

void DriverMotorInterface::Shutdown() noexcept
{
  // 속도 0을 보내고 이후 명령을 막는다. 토크는 드라이버 watchdog(3 s)이 끈다
  try {
    std::lock_guard<std::mutex> lock(command_mutex_);
    is_command_blocked_ = true;
    PublishWheelCommand(WheelCommand{});
  } catch (...) {
  }
}

void DriverMotorInterface::StatesCallback(const lekiwi_msgs::msg::MotorStateArray::SharedPtr msg)
{
  const rclcpp::Time receipt_time = clock_->now();
  std::lock_guard<std::mutex> lock(states_mutex_);
  latest_states_ = msg;
  last_states_time_ = receipt_time;
  // 토크를 켠 뒤 guard 시각보다 늦게 찍힌 메시지에서만 드라이버의 토크 상태를 본다
  const int64_t stamp_nanoseconds = rclcpp::Time(msg->header.stamp).nanoseconds();
  const bool is_torque_checked =
    torque_guard_stamp_nanoseconds_ && stamp_nanoseconds > *torque_guard_stamp_nanoseconds_;
  bool is_new_torque_off = false;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    for (const auto & motor : msg->motors) {
      if (motor.id == config_.wheel_ids[i]) {
        if (motor.online) {
          last_online_times_[i] = receipt_time;
        }
        if (is_torque_checked && !motor.torque_enabled && !is_driver_torque_off_[i]) {
          is_driver_torque_off_[i] = true;
          is_new_torque_off = true;
        }
        break;
      }
    }
  }
  if (is_new_torque_off) {
    RCLCPP_ERROR(
      logger_, "motor driver reports wheel torque off while motor power is on; treating as link loss");
  }
}

bool DriverMotorInterface::IsConnected() const
{
  const rclcpp::Time now = clock_->now();
  std::lock_guard<std::mutex> lock(states_mutex_);
  if (!last_states_time_ || (now - *last_states_time_).seconds() > config_.driver_states_timeout) {
    return false;
  }
  for (const bool is_torque_off : is_driver_torque_off_) {
    if (is_torque_off) {
      return false;
    }
  }
  for (const auto & last_online_time : last_online_times_) {
    if (!last_online_time ||
      (now - *last_online_time).seconds() > config_.driver_states_timeout)
    {
      return false;
    }
  }
  return true;
}

std::optional<WheelFeedbackSample> DriverMotorInterface::ReadWheelFeedback()
{
  HandleLinkLoss();

  lekiwi_msgs::msg::MotorStateArray::SharedPtr states;
  {
    std::lock_guard<std::mutex> lock(states_mutex_);
    states = latest_states_;
  }
  if (!states) {
    return std::nullopt;
  }
  const WheelStatesFrame frame = ExtractWheelStatesFrame(*states, config_.wheel_ids);
  // 같은 메시지를 두 번 쓰지 않는다
  if (last_consumed_stamp_nanoseconds_ &&
    frame.stamp_nanoseconds <= *last_consumed_stamp_nanoseconds_)
  {
    return std::nullopt;
  }
  last_consumed_stamp_nanoseconds_ = frame.stamp_nanoseconds;

  const auto sample = estimator_.Update(frame);
  if (config_.is_encoder_read_logged) {
    LogEncoderRead(frame, sample);
  }
  return sample;
}

bool DriverMotorInterface::SetMotorPower(const bool is_enabled)
{
  // 켜기 · 끄기 모두 위치 추정 기준점을 새로 잡는다
  estimator_.Reset();
  last_consumed_stamp_nanoseconds_.reset();
  is_link_loss_handled_ = false;

  if (!is_enabled) {
    // 속도 0을 먼저 보낸 뒤 토크를 끈다
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      PublishWheelCommand(WheelCommand{});
    }
    const bool is_disabled = CallWheelTorque(false);
    if (is_disabled) {
      SetMotorPowerState(false);
      SetTorqueGuard(std::nullopt, true);
    }
    // 끄기를 확인하지 못하면 명령을 막아 드라이버 watchdog이 토크를 끄게 한다
    SetCommandBlocked(!is_disabled);
    return is_disabled;
  }

  if (!CallWheelTorque(true)) {
    return false;
  }
  const rclcpp::Time guard_time =
    clock_->now() + rclcpp::Duration::from_seconds(torque_guard_seconds);
  SetTorqueGuard(guard_time.nanoseconds(), true);
  SetCommandBlocked(false);
  SetMotorPowerState(true);
  return true;
}

void DriverMotorInterface::WriteClampedWheelSpeeds(const WheelCommand & command)
{
  HandleLinkLoss();
  if (!IsMotorPowerEnabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (is_command_blocked_) {
    return;
  }
  PublishWheelCommand(command);
}

void DriverMotorInterface::PublishWheelCommand(const WheelCommand & command)
{
  pub_wheel_command_->publish(
    MakeWheelCommandMessage(
      command, config_.wheel_ids, config_.feedback.wheel_directions, clock_->now()));
}

void DriverMotorInterface::SetCommandBlocked(const bool is_blocked)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  is_command_blocked_ = is_blocked;
}

void DriverMotorInterface::SetTorqueGuard(
  const std::optional<int64_t> guard_stamp_nanoseconds,
  const bool is_clearing_torque_off)
{
  std::lock_guard<std::mutex> lock(states_mutex_);
  torque_guard_stamp_nanoseconds_ = guard_stamp_nanoseconds;
  if (is_clearing_torque_off) {
    is_driver_torque_off_.fill(false);
  }
}

void DriverMotorInterface::HandleLinkLoss()
{
  // 토크가 켜진 동안 처음 끊김을 발견했을 때 한 번만 처리한다
  if (!IsMotorPowerEnabled() || is_link_loss_handled_ || IsConnected()) {
    return;
  }
  is_link_loss_handled_ = true;
  RCLCPP_ERROR(
    logger_, "motor driver link lost (%s); writing zero speed and disabling wheel torque",
    DescribeLink().c_str());
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    PublishWheelCommand(WheelCommand{});
  }
  estimator_.Reset();
  if (CallWheelTorque(false)) {
    SetMotorPowerState(false);
    // 토크가 꺼졌으므로 더 보지 않는다. 고장 표시는 다음 전원 요청까지 남긴다
    SetTorqueGuard(std::nullopt, false);
    return;
  }
  SetCommandBlocked(true);
  RCLCPP_ERROR(
    logger_,
    "wheel torque disable not confirmed; stopped sending wheel commands so the driver watchdog "
    "disables torque");
}

bool DriverMotorInterface::CallWheelTorque(const bool is_enabled)
{
  using SteadyClock = std::chrono::steady_clock;
  const auto deadline = SteadyClock::now() +
    std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(config_.torque_service_timeout));
  const auto remaining = [&deadline]() {
      return std::max(SteadyClock::duration::zero(), deadline - SteadyClock::now());
    };
  const char * action = is_enabled ? "enable" : "disable";

  try {
    if (!torque_client_->wait_for_service(remaining())) {
      RCLCPP_ERROR(
        logger_, "wheel torque %s failed: service %s is not available within %.2f s",
        action, torque_client_->get_service_name(), config_.torque_service_timeout);
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = is_enabled;
    auto future = torque_client_->async_send_request(request);
    if (future.wait_for(remaining()) != std::future_status::ready) {
      torque_client_->remove_pending_request(future);
      RCLCPP_ERROR(
        logger_, "wheel torque %s failed: no response from %s within %.2f s",
        action, torque_client_->get_service_name(), config_.torque_service_timeout);
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(
        logger_, "wheel torque %s refused by motor driver: %s", action, response->message.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "wheel torque %s: %s", action, response->message.c_str());
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "wheel torque %s failed: %s", action, error.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "wheel torque %s failed with an unknown error", action);
  }
  return false;
}

std::string DriverMotorInterface::DescribeLink() const
{
  const rclcpp::Time now = clock_->now();
  std::lock_guard<std::mutex> lock(states_mutex_);
  char buffer[64];
  std::string text;
  if (last_states_time_) {
    std::snprintf(buffer, sizeof(buffer), "states age %.3f s", (now - *last_states_time_).seconds());
    text = buffer;
  } else {
    text = "no states received";
  }
  std::string stale_ids;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    const auto & last_online_time = last_online_times_[i];
    if (!last_online_time ||
      (now - *last_online_time).seconds() > config_.driver_states_timeout)
    {
      stale_ids += (stale_ids.empty() ? "" : ", ") + std::to_string(config_.wheel_ids[i]);
    }
  }
  if (!stale_ids.empty()) {
    text += ", wheel ids [" + stale_ids + "] not online";
  }
  std::string torque_off_ids;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    if (is_driver_torque_off_[i]) {
      torque_off_ids += (torque_off_ids.empty() ? "" : ", ") + std::to_string(config_.wheel_ids[i]);
    }
  }
  if (!torque_off_ids.empty()) {
    text += ", driver reports torque off for wheel ids [" + torque_off_ids + "]";
  }
  return text;
}

void DriverMotorInterface::LogEncoderRead(
  const WheelStatesFrame & frame,
  const std::optional<WheelFeedbackSample> & sample) const
{
  const auto & diagnostics = estimator_.GetLastDiagnostics();
  const bool is_position = config_.feedback.source == EncoderFeedbackSource::Position;
  const std::array<double, WHEEL_COUNT> speeds = sample ?
    std::array<double, WHEEL_COUNT>{
    sample->wheel_speeds.left, sample->wheel_speeds.back, sample->wheel_speeds.right} :
    std::array<double, WHEEL_COUNT>{};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    if (is_position) {
      RCLCPP_INFO(
        logger_,
        "encoder position wheel_id=%u online=%d raw_position_ticks=%d previous_position_ticks=%d "
        "raw_delta_ticks=%d window_delta_ticks=%d window_elapsed_seconds=%.6f wheel_speed=%.6f "
        "sample=%d",
        config_.wheel_ids[i], frame.is_online[i], frame.position_ticks[i],
        diagnostics.previous_ticks[i], diagnostics.delta_ticks[i],
        diagnostics.window_delta_ticks[i], diagnostics.window_elapsed_seconds, speeds[i],
        sample.has_value());
    } else {
      RCLCPP_INFO(
        logger_, "encoder speed wheel_id=%u online=%d raw_velocity=%.6f wheel_speed=%.6f sample=%d",
        config_.wheel_ids[i], frame.is_online[i], frame.velocity[i], speeds[i],
        sample.has_value());
    }
  }
}

}  // namespace lekiwi_base_controller
