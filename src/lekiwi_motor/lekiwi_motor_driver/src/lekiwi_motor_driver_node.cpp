#include "lekiwi_motor_driver/lekiwi_motor_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "lekiwi_motor_driver/sts3215.hpp"

namespace lekiwi_motor_driver
{

using lekiwi_msgs::msg::MotorState;

namespace
{

// "1, 2, 3" (로그 · 서비스 메시지용)
template<typename Ids>
std::string JoinIds(const Ids & ids)
{
  std::string text;
  for (const uint8_t id : ids) {
    text += (text.empty() ? "" : ", ") + std::to_string(id);
  }
  return text;
}

}  // namespace

LekiwiMotorDriverNode::LekiwiMotorDriverNode()
: Node("lekiwi_motor_driver_node")
{
  // 필요한 파라미터 선언 (기본값 설정) → 로드·검증
  DeclareParams();
  InitializeParams();

  // 버스 열기. 실패하면 예외가 main까지 올라가 노드를 종료한다.
  bus_ = std::make_unique<FeetechBus>(port_, baud_rate_, read_timeout_ms_);
  bus_->Open();
  const char * write_mode = is_wheel_write_enabled_ ?
    (is_arm_write_enabled_ ? "wheel and arm write enabled" : "wheel write enabled, arm read only") :
    (is_arm_write_enabled_ ? "arm write enabled, wheel read only" :
    "read only: no torque or motion commands");
  RCLCPP_INFO(
    get_logger(), "Feetech bus open on %s at %d baud (%s)", port_.c_str(), baud_rate_, write_mode);
  if (!bus_->exclusive()) {
    RCLCPP_WARN(
      get_logger(), "could not lock %s for exclusive use; do not run another motor program",
      port_.c_str());
  }

  pub_states_ = create_publisher<lekiwi_msgs::msg::MotorStateArray>(
    states_topic_, rclcpp::SensorDataQoS());

  ProbeMotors();
  last_slow_read_ = Clock::now();
  slow_slot_interval_ = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(slow_read_period_ / static_cast<double>(slots_.size())));
  next_slow_slot_time_ = last_slow_read_ + slow_slot_interval_;

  if (is_wheel_write_enabled_) {
    // 다른 프로그램이 남긴 바퀴 토크 · 속도를 정리하고 토크가 꺼진 상태에서 시작한다.
    // 응답하지 않는 바퀴는 느린 주기마다 다시 끄고, 확인될 때까지 토크 켜기를 거부한다.
    std::string message;
    if (DisableWheelTorque(message)) {
      RCLCPP_INFO(get_logger(), "startup: %s", message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "startup: %s", message.c_str());
    }
    CreateWheelInterfaces();
  }
  if (is_arm_write_enabled_) {
    // 팔 토크는 시작할 때 건드리지 않는다 (켜져 있던 팔이 떨어지지 않게). 명령은 서비스로 켠 뒤에만 쓴다.
    CreateArmInterfaces();
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {TimerCallback();});

  RCLCPP_INFO(
    get_logger(), "publishing %s at %.1f Hz", pub_states_->get_topic_name(), publish_rate_hz_);
}

LekiwiMotorDriverNode::~LekiwiMotorDriverNode()
{
  // 종료할 때 토크가 켜져 있으면 바퀴를 세우고 토크를 끈다.
  if (!bus_ || (!is_wheel_torque_on_ && !has_pending_torque_off_)) {
    return;
  }
  try {
    std::string message;
    if (DisableWheelTorque(message)) {
      RCLCPP_INFO(get_logger(), "shutdown: wheel speed zeroed and torque disabled");
    } else {
      RCLCPP_ERROR(get_logger(), "shutdown: %s", message.c_str());
    }
  } catch (...) {
  }
}

void LekiwiMotorDriverNode::DeclareParams()
{
  // 버스 연결
  declare_parameter<std::string>("port", "/dev/ttyACM0");
  declare_parameter<int64_t>("baud_rate", 1000000);
  declare_parameter<int64_t>("read_timeout_ms", 10);
  // 모터 (LeRobot LeKiwi 설정과 같은 ID · 이름)
  declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4, 5, 6, 7, 8, 9});
  declare_parameter<std::vector<std::string>>(
    "motor_names", {
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex",
      "arm_wrist_roll", "arm_gripper", "base_left_wheel", "base_back_wheel", "base_right_wheel"});
  // 주기 · 장애 처리
  declare_parameter<double>("publish_rate_hz", 30.0);
  declare_parameter<double>("slow_read_period", 1.0);
  declare_parameter<int64_t>("offline_after_failures", 5);
  // 토픽
  declare_parameter<std::string>("states_topic", "lekiwi/motor/states");
  // 바퀴 쓰기 (기본 꺼짐)
  declare_parameter<bool>("wheel_write_enabled", false);
  declare_parameter<std::vector<int64_t>>("wheel_ids", {7, 8, 9});
  declare_parameter<double>("speed_tick_scale", 651.8986469044033);
  declare_parameter<int64_t>("speed_tick_limit", 1500);
  declare_parameter<double>("wheel_command_timeout", 0.5);
  declare_parameter<double>("wheel_torque_off_timeout", 3.0);
  declare_parameter<std::string>("wheel_command_topic", "lekiwi/motor/wheel_command");
  declare_parameter<std::string>("wheel_torque_service", "lekiwi/motor/wheel_torque");
  // 팔 쓰기 (기본 꺼짐)
  declare_parameter<bool>("arm_write_enabled", false);
  declare_parameter<std::vector<int64_t>>("arm_ids", {1, 2, 3, 4, 5, 6});
  declare_parameter<int64_t>("arm_max_goal_offset_ticks", 1024);
  declare_parameter<std::string>("arm_command_topic", "lekiwi/motor/arm_command");
  declare_parameter<std::string>("arm_torque_service", "lekiwi/motor/arm_torque");
  declare_parameter<std::string>("arm_ready_topic", "lekiwi/motor/arm_ready");
  declare_parameter<std::string>("arm_registers_topic", "lekiwi/motor/arm_registers");
}

void LekiwiMotorDriverNode::InitializeParams()
{
  port_ = get_parameter("port").as_string();
  baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
  const int64_t read_timeout_ms = get_parameter("read_timeout_ms").as_int();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  slow_read_period_ = get_parameter("slow_read_period").as_double();
  offline_after_failures_ = get_parameter("offline_after_failures").as_int();
  states_topic_ = get_parameter("states_topic").as_string();

  if (port_.empty() || states_topic_.empty()) {
    throw std::invalid_argument("port and states_topic must not be empty");
  }
  if (read_timeout_ms < 1 || read_timeout_ms > 100) {
    throw std::invalid_argument("read_timeout_ms must be in [1, 100]");
  }
  read_timeout_ms_ = static_cast<int>(read_timeout_ms);
  if (publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 100.0) {
    throw std::invalid_argument("publish_rate_hz must be in (0, 100]");
  }
  if (slow_read_period_ <= 0.0) {
    throw std::invalid_argument("slow_read_period must be > 0");
  }
  if (offline_after_failures_ < 1) {
    throw std::invalid_argument("offline_after_failures must be >= 1");
  }

  const auto ids = get_parameter("motor_ids").as_integer_array();
  const auto names = get_parameter("motor_names").as_string_array();
  if (ids.empty() || ids.size() != names.size()) {
    throw std::invalid_argument("motor_ids and motor_names must be non-empty and the same length");
  }
  std::set<int64_t> unique_ids;
  std::set<std::string> unique_names;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (ids[i] < 0 || ids[i] > 252) {
      throw std::invalid_argument("motor_ids must be in [0, 252]");
    }
    if (names[i].empty()) {
      throw std::invalid_argument("motor_names must not be empty");
    }
    if (!unique_ids.insert(ids[i]).second || !unique_names.insert(names[i]).second) {
      throw std::invalid_argument("motor_ids and motor_names must be unique");
    }
    MotorSlot slot;
    slot.state.id = static_cast<uint8_t>(ids[i]);
    slot.state.name = names[i];
    slot.state.mode = MotorState::MODE_UNKNOWN;
    slots_.push_back(slot);
  }

  // 바퀴 쓰기. 쓰기를 끈 상태에서도 값은 검증해 설정 실수를 일찍 알린다.
  is_wheel_write_enabled_ = get_parameter("wheel_write_enabled").as_bool();
  speed_tick_scale_ = get_parameter("speed_tick_scale").as_double();
  const int64_t speed_tick_limit = get_parameter("speed_tick_limit").as_int();
  wheel_command_timeout_ = get_parameter("wheel_command_timeout").as_double();
  wheel_torque_off_timeout_ = get_parameter("wheel_torque_off_timeout").as_double();
  wheel_command_topic_ = get_parameter("wheel_command_topic").as_string();
  wheel_torque_service_ = get_parameter("wheel_torque_service").as_string();

  if (!std::isfinite(speed_tick_scale_) || speed_tick_scale_ <= 0.0) {
    throw std::invalid_argument("speed_tick_scale must be positive and finite");
  }
  if (speed_tick_limit < 0 || speed_tick_limit > sts3215::kMaxSpeedMagnitude) {
    throw std::invalid_argument("speed_tick_limit must be in [0, 32767]");
  }
  speed_tick_limit_ = static_cast<int>(speed_tick_limit);
  if (!std::isfinite(wheel_command_timeout_) || wheel_command_timeout_ <= 0.0) {
    throw std::invalid_argument("wheel_command_timeout must be positive and finite");
  }
  if (!std::isfinite(wheel_torque_off_timeout_) ||
    wheel_torque_off_timeout_ < wheel_command_timeout_)
  {
    throw std::invalid_argument("wheel_torque_off_timeout must be >= wheel_command_timeout");
  }
  if (wheel_command_topic_.empty() || wheel_torque_service_.empty()) {
    throw std::invalid_argument("wheel_command_topic and wheel_torque_service must not be empty");
  }

  const auto wheel_ids = get_parameter("wheel_ids").as_integer_array();
  if (wheel_ids.empty()) {
    throw std::invalid_argument("wheel_ids must not be empty");
  }
  std::set<int64_t> unique_wheel_ids;
  for (const int64_t wheel_id : wheel_ids) {
    if (unique_ids.count(wheel_id) == 0) {
      throw std::invalid_argument("wheel_ids must be a subset of motor_ids");
    }
    if (!unique_wheel_ids.insert(wheel_id).second) {
      throw std::invalid_argument("wheel_ids must be unique");
    }
    wheel_ids_.push_back(static_cast<uint8_t>(wheel_id));
  }

  // 팔 쓰기. 켰을 때만 읽고 검증한다 (팔 쓰기를 끈 설정의 arm_ids가 읽기 전용 시작을 막지 않게).
  is_arm_write_enabled_ = get_parameter("arm_write_enabled").as_bool();
  if (!is_arm_write_enabled_) {
    return;
  }
  arm_command_topic_ = get_parameter("arm_command_topic").as_string();
  arm_torque_service_ = get_parameter("arm_torque_service").as_string();
  arm_ready_topic_ = get_parameter("arm_ready_topic").as_string();
  arm_registers_topic_ = get_parameter("arm_registers_topic").as_string();
  if (arm_command_topic_.empty() || arm_torque_service_.empty() || arm_ready_topic_.empty() ||
    arm_registers_topic_.empty())
  {
    throw std::invalid_argument("arm topic and service names must not be empty");
  }
  const int64_t arm_max_goal_offset_ticks = get_parameter("arm_max_goal_offset_ticks").as_int();
  if (arm_max_goal_offset_ticks < 1 || arm_max_goal_offset_ticks > sts3215::kMaxPositionTicks) {
    throw std::invalid_argument("arm_max_goal_offset_ticks must be in [1, 4095]");
  }
  arm_max_goal_offset_ticks_ = static_cast<int>(arm_max_goal_offset_ticks);

  const auto arm_ids = get_parameter("arm_ids").as_integer_array();
  if (arm_ids.empty()) {
    throw std::invalid_argument("arm_ids must not be empty");
  }
  std::set<int64_t> unique_arm_ids;
  for (const int64_t arm_id : arm_ids) {
    if (unique_ids.count(arm_id) == 0) {
      throw std::invalid_argument("arm_ids must be a subset of motor_ids");
    }
    if (unique_wheel_ids.count(arm_id) != 0) {
      throw std::invalid_argument("arm_ids must not overlap wheel_ids");
    }
    if (!unique_arm_ids.insert(arm_id).second) {
      throw std::invalid_argument("arm_ids must be unique");
    }
    arm_ids_.push_back(static_cast<uint8_t>(arm_id));
  }
}

void LekiwiMotorDriverNode::ProbeMotors()
{
  std::size_t found = 0;
  for (auto & slot : slots_) {
    const auto & state = slot.state;
    std::vector<uint8_t> data;
    uint8_t error = 0;
    if (!bus_->Read(
        state.id, sts3215::kAddrModelNumber, sts3215::kModelNumberLength, data, error))
    {
      RCLCPP_WARN(get_logger(), "motor %u (%s): no response", state.id, state.name.c_str());
      continue;
    }
    const uint16_t model = sts3215::Word(data.data());
    RCLCPP_INFO(
      get_logger(), "motor %u (%s): model %u%s", state.id, state.name.c_str(), model,
      model == sts3215::kModelNumber ? " (STS3215)" : " (not STS3215, values may be wrong)");
    slot.responsive = true;
    ++found;
    ReadConfig(slot);
  }

  if (found == 0) {
    RCLCPP_WARN(
      get_logger(),
      "no motor responded: check motor power (battery), port and baud_rate; retrying every %.1f s",
      slow_read_period_);
  } else {
    RCLCPP_INFO(get_logger(), "%zu/%zu motors responded", found, slots_.size());
  }
}

void LekiwiMotorDriverNode::TimerCallback()
{
  const auto start = Clock::now();
  const bool slow_cycle =
    start - last_slow_read_ >= std::chrono::duration<double>(slow_read_period_);
  if (slow_cycle) {
    last_slow_read_ = start;
  }

  // 모터마다 slow_read_period에 한 번 하는 느린 읽기(운전 모드 · 토크, 응답 없는 모터 다시 찾기)는
  // 모터 수로 나눠 주기마다 많아야 한 모터만 한다. 한 주기에 모두 몰면 30 Hz 주기를 넘는다 (README 주기 예산).
  const MotorSlot * slow_slot = nullptr;
  if (start >= next_slow_slot_time_) {
    slow_slot = &slots_[next_slow_slot_];
    next_slow_slot_ = (next_slow_slot_ + 1) % slots_.size();
    // 밀렸으면 쌓아 두지 않는다 (주기마다 하나씩)
    next_slow_slot_time_ = std::max(next_slow_slot_time_ + slow_slot_interval_, start);
  }
  // 읽기가 실패한 주기는 read_timeout_ms를 기다렸다: 주기를 넘겨도 경고하지 않는다 (주기를 낮춰도 안 낫는다)
  bool has_failed_read = false;
  for (auto & slot : slots_) {
    // 응답하지 않는 모터는 막 끊긴 뒤 kOfflineRetryGrace 동안 주기마다, 그 뒤에는 자기 느린 읽기 차례에만
    // 다시 시도한다. 짧은 끊김은 바로 돌아오고, 오래 끊긴 모터가 다른 모터 읽기를 늦추지 않는다.
    const bool is_slow_slot = &slot == slow_slot;
    const bool was_responsive = slot.responsive;
    if (was_responsive || is_slow_slot || start - slot.offline_since < kOfflineRetryGrace) {
      const bool ok = ReadPresent(slot);
      HandleReadResult(slot, ok);
      has_failed_read = has_failed_read || !ok;
      // 다시 응답한 모터는 운전 모드 · 토크를 바로 읽는다 (끊길 때 MODE_UNKNOWN · false로 지웠다)
      if (ok && (is_slow_slot || !was_responsive)) {
        ReadConfig(slot);
      }
    }
  }
  // 주기 시각은 모터를 다 읽은 때다 (아래 느린 일 · 감시 전)
  const auto stamp = now();

  // 이번 주기에 느린 일(확인 못 한 토크 끄기 다시 보내기 · 읽지 못한 팔 EEPROM 읽기)을 하면 경고하지 않는다
  bool did_slow_work = false;

  if (is_wheel_write_enabled_) {
    if (has_pending_torque_off_ && slow_cycle) {
      did_slow_work = true;
      std::string message;
      if (WriteTorque(wheel_ids_, "wheel", false, message)) {
        has_pending_torque_off_ = false;
        RCLCPP_INFO(get_logger(), "wheel torque disable confirmed");
      }
    }
    SuperviseWheels();
  }
  if (is_arm_write_enabled_ && slow_cycle) {
    did_slow_work = did_slow_work || !arm_pending_torque_off_.empty() ||
      std::find(arm_registers_.valid.begin(), arm_registers_.valid.end(), false) !=
      arm_registers_.valid.end();
    RetryArmTorqueOff();
    // 시작할 때 응답하지 않던 팔이 응답하면 EEPROM 설정을 읽어 다시 발행한다
    if (ReadArmRegisters()) {
      arm_registers_.header.stamp = now();
      pub_arm_registers_->publish(arm_registers_);
    }
  }

  lekiwi_msgs::msg::MotorStateArray message;
  message.header.stamp = stamp;
  message.motors.reserve(slots_.size());
  for (const auto & slot : slots_) {
    message.motors.push_back(slot.state);
  }
  pub_states_->publish(message);

  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  const double period_ms = 1000.0 / publish_rate_hz_;
  if (!did_slow_work && !has_failed_read && elapsed_ms > period_ms) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "reading motors took %.1f ms, longer than the %.1f ms period; lower publish_rate_hz",
      elapsed_ms, period_ms);
  }
}

bool LekiwiMotorDriverNode::ReadPresent(MotorSlot & slot)
{
  std::vector<uint8_t> data;
  uint8_t error = 0;
  if (!bus_->Read(
      slot.state.id, sts3215::kPresentBlockAddress, sts3215::kPresentBlockLength, data, error))
  {
    return false;
  }
  const sts3215::PresentState present = sts3215::DecodePresentBlock(data);
  auto & state = slot.state;
  state.online = true;
  state.position_ticks = present.position_ticks;
  state.position = sts3215::TicksToRad(present.position_ticks - sts3215::kCenterTicks);
  state.velocity = sts3215::TicksToRad(present.velocity_ticks);
  state.load = present.load_permille * 0.1;
  state.voltage = present.voltage;
  state.temperature = present.temperature;
  state.current = present.current;
  state.moving = present.moving;
  state.status = present.status;
  return true;
}

bool LekiwiMotorDriverNode::ReadConfig(MotorSlot & slot)
{
  std::vector<uint8_t> data;
  uint8_t error = 0;
  if (!bus_->Read(
      slot.state.id, sts3215::kConfigBlockAddress, sts3215::kConfigBlockLength, data, error))
  {
    return false;
  }
  const uint8_t mode = data[0];
  slot.state.mode = mode <= MotorState::MODE_STEP ? mode : MotorState::MODE_UNKNOWN;
  slot.state.torque_enabled =
    data[sts3215::kAddrTorqueEnable - sts3215::kConfigBlockAddress] != 0;
  return true;
}

void LekiwiMotorDriverNode::HandleReadResult(MotorSlot & slot, const bool ok)
{
  auto & state = slot.state;
  if (ok) {
    if (!slot.responsive) {
      RCLCPP_INFO(get_logger(), "motor %u (%s) is responding", state.id, state.name.c_str());
    }
    slot.responsive = true;
    slot.consecutive_failures = 0;
    ReportStatusChange(slot);
    return;
  }

  ClearReadings(state);
  ++slot.consecutive_failures;
  if (slot.responsive && slot.consecutive_failures >= offline_after_failures_) {
    slot.responsive = false;
    slot.offline_since = Clock::now();
    state.mode = MotorState::MODE_UNKNOWN;
    state.torque_enabled = false;
    RCLCPP_WARN(
      get_logger(),
      "motor %u (%s) stopped responding after %ld failed reads; retrying every cycle for %.1f s, "
      "then every %.1f s", state.id, state.name.c_str(), static_cast<long>(slot.consecutive_failures),
      std::chrono::duration<double>(kOfflineRetryGrace).count(), slow_read_period_);
  }
}

void LekiwiMotorDriverNode::ReportStatusChange(MotorSlot & slot)
{
  const auto & state = slot.state;
  if (state.status == slot.last_status) {
    return;
  }
  if (state.status != 0) {
    RCLCPP_WARN(
      get_logger(), "motor %u (%s) status 0x%02X: %s", state.id, state.name.c_str(),
      state.status, sts3215::StatusToString(state.status).c_str());
  } else {
    RCLCPP_INFO(get_logger(), "motor %u (%s) status back to normal", state.id, state.name.c_str());
  }
  slot.last_status = state.status;
}

void LekiwiMotorDriverNode::ClearReadings(MotorState & state)
{
  state.online = false;
  state.position_ticks = 0;
  state.position = 0.0;
  state.velocity = 0.0;
  state.load = 0.0;
  state.voltage = 0.0;
  state.temperature = 0.0;
  state.current = 0.0;
  state.moving = false;
  state.status = 0;
}

// ---------------------------------------------------------------------------
// 바퀴 쓰기
// ---------------------------------------------------------------------------

void LekiwiMotorDriverNode::CreateWheelInterfaces()
{
  // 최신 명령만 의미가 있으므로 깊이 1
  sub_wheel_command_ = create_subscription<lekiwi_msgs::msg::WheelCommand>(
    wheel_command_topic_, rclcpp::QoS(1),
    [this](const lekiwi_msgs::msg::WheelCommand::SharedPtr msg) {WheelCommandCallback(msg);});
  srv_wheel_torque_ = create_service<std_srvs::srv::SetBool>(
    wheel_torque_service_,
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      WheelTorqueCallback(request, response);
    });

  RCLCPP_INFO(
    get_logger(),
    "wheel write ready for ids [%s]: %s (rad/s), %s (SetBool); torque stays off until requested",
    JoinIds(wheel_ids_).c_str(), sub_wheel_command_->get_topic_name(),
    srv_wheel_torque_->get_service_name());
}

void LekiwiMotorDriverNode::WheelCommandCallback(
  const lekiwi_msgs::msg::WheelCommand::SharedPtr msg)
{
  if (!is_wheel_torque_on_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "wheel command ignored: wheel torque is off");
    return;
  }
  if (msg->ids.empty() || msg->ids.size() != msg->velocity.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "wheel command ignored: ids and velocity sizes differ");
    return;
  }

  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> entries;
  std::set<uint8_t> seen;
  for (std::size_t i = 0; i < msg->ids.size(); ++i) {
    const uint8_t id = msg->ids[i];
    // 팔 등 바퀴가 아닌 ID가 하나라도 있으면 메시지 전체를 버린다.
    if (std::find(wheel_ids_.begin(), wheel_ids_.end(), id) == wheel_ids_.end() ||
      !seen.insert(id).second)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "wheel command ignored: id %u is not a configured wheel or is repeated", id);
      return;
    }
    if (!std::isfinite(msg->velocity[i])) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "wheel command ignored: non-finite velocity");
      return;
    }
    const int ticks = sts3215::SpeedToTicks(msg->velocity[i], speed_tick_scale_, speed_tick_limit_);
    entries.emplace_back(id, sts3215::EncodeGoalSpeed(ticks));
  }

  if (!bus_->SyncWrite(sts3215::kAddrGoalSpeed, 2, entries)) {
    RCLCPP_ERROR(get_logger(), "failed to write wheel speeds; stopping wheels and disabling torque");
    std::string message;
    DisableWheelTorque(message);
    return;
  }
  last_wheel_command_ = Clock::now();
  if (is_command_stale_) {
    is_command_stale_ = false;
    RCLCPP_INFO(get_logger(), "wheel commands resumed");
  }
}

void LekiwiMotorDriverNode::WheelTorqueCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::string message;
  response->success = request->data ? EnableWheelTorque(message) : DisableWheelTorque(message);
  response->message = message;
  if (response->success) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }
}

void LekiwiMotorDriverNode::SuperviseWheels()
{
  if (!is_wheel_torque_on_) {
    return;
  }

  std::string message;
  for (const MotorSlot * slot : MotorSlots(wheel_ids_)) {
    const auto & state = slot->state;
    std::string reason;
    if (!slot->responsive) {
      reason = "stopped responding";
    } else if (state.online && (state.status & sts3215::kStatusFaultMask) != 0) {
      reason = "reports " + sts3215::StatusToString(state.status);
    } else if (state.online && !state.torque_enabled) {
      reason = "torque turned off outside this driver";
    }
    if (!reason.empty()) {
      RCLCPP_ERROR(
        get_logger(), "wheel %u (%s) %s; stopping all wheels and disabling torque",
        state.id, state.name.c_str(), reason.c_str());
      DisableWheelTorque(message);
      return;
    }
  }

  const double age =
    std::chrono::duration<double>(Clock::now() - last_wheel_command_).count();
  if (age > wheel_torque_off_timeout_) {
    RCLCPP_ERROR(
      get_logger(), "no wheel command for %.1f s; disabling wheel torque", age);
    DisableWheelTorque(message);
    return;
  }
  if (age > wheel_command_timeout_) {
    if (!is_command_stale_) {
      is_command_stale_ = true;
      RCLCPP_WARN(get_logger(), "no wheel command for %.2f s; writing zero speed", age);
    }
    WriteZeroWheelSpeed();
  }
}

bool LekiwiMotorDriverNode::EnableWheelTorque(std::string & message)
{
  if (has_pending_torque_off_) {
    message = "refused: an earlier wheel torque disable is not confirmed yet";
    return false;
  }

  // 켜기 전에 모든 바퀴가 응답하고, 속도 모드이고, 보호 비트가 꺼져 있는지 지금 다시 읽는다.
  for (MotorSlot * slot : MotorSlots(wheel_ids_)) {
    auto & state = slot->state;
    const std::string wheel = "wheel " + std::to_string(state.id) + " (" + state.name + ")";
    if (!slot->responsive || !ReadConfig(*slot) || !ReadPresent(*slot)) {
      message = "refused: " + wheel + " is not responding";
      return false;
    }
    if (state.mode != sts3215::kModeVelocity) {
      message = "refused: " + wheel + " is in mode " + std::to_string(state.mode) +
        ", not velocity mode (1)";
      return false;
    }
    if ((state.status & sts3215::kStatusFaultMask) != 0) {
      message = "refused: " + wheel + " reports " + sts3215::StatusToString(state.status);
      return false;
    }
  }

  if (!WriteZeroWheelSpeed()) {
    message = "refused: failed to write zero wheel speed";
    return false;
  }
  std::string torque_message;
  bool is_enabled = WriteTorque(wheel_ids_, "wheel", true, torque_message);
  // 다시 읽어 실제로 켜졌는지 확인한다.
  for (MotorSlot * slot : MotorSlots(wheel_ids_)) {
    if (!is_enabled) {
      break;
    }
    if (!ReadConfig(*slot) || !slot->state.torque_enabled) {
      is_enabled = false;
      torque_message = "wheel " + std::to_string(slot->state.id) + " torque did not read back on";
    }
  }
  if (!is_enabled) {
    std::string disable_message;
    DisableWheelTorque(disable_message);
    message = "failed to enable wheel torque: " + torque_message;
    return false;
  }

  is_wheel_torque_on_ = true;
  is_command_stale_ = false;
  last_wheel_command_ = Clock::now();
  message = "wheel torque enabled (speed 0)";
  return true;
}

bool LekiwiMotorDriverNode::DisableWheelTorque(std::string & message)
{
  // 명령을 먼저 막고, 속도 0 → 토크 끔 순서로 보낸다. 하나가 실패해도 나머지는 보낸다.
  is_wheel_torque_on_ = false;
  is_command_stale_ = false;
  const bool is_zeroed = WriteZeroWheelSpeed();
  std::string torque_message;
  const bool is_disabled = WriteTorque(wheel_ids_, "wheel", false, torque_message);
  has_pending_torque_off_ = !is_disabled;
  if (is_zeroed && is_disabled) {
    message = "wheel speed zeroed and torque disabled";
    return true;
  }
  message = std::string(is_zeroed ? "" : "failed to write zero wheel speed; ") +
    (is_disabled ? "" : torque_message + "; retrying torque disable every slow cycle");
  return false;
}

bool LekiwiMotorDriverNode::WriteZeroWheelSpeed()
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> entries;
  for (const uint8_t id : wheel_ids_) {
    entries.emplace_back(id, sts3215::EncodeGoalSpeed(0));
  }
  return bus_->SyncWrite(sts3215::kAddrGoalSpeed, 2, entries);
}

bool LekiwiMotorDriverNode::WriteTorque(
  const std::vector<uint8_t> & ids, const std::string & label, const bool enable,
  std::string & message)
{
  const std::vector<uint8_t> failed = WriteTorqueIds(ids, enable);
  if (!failed.empty()) {
    message = std::string("no torque ") + (enable ? "enable" : "disable") +
      " acknowledgement from " + label + " ids [" + JoinIds(failed) + "]";
    return false;
  }
  return true;
}

std::vector<uint8_t> LekiwiMotorDriverNode::WriteTorqueIds(
  const std::vector<uint8_t> & ids, const bool enable)
{
  std::vector<uint8_t> failed;
  for (MotorSlot * slot : MotorSlots(ids)) {
    uint8_t error = 0;
    const bool is_acknowledged = bus_->Write(
      slot->state.id, sts3215::kAddrTorqueEnable, {static_cast<uint8_t>(enable ? 1 : 0)}, error);
    // 켤 때는 응답의 상태 바이트도 0이어야 한다. 끌 때는 응답만 오면 된다.
    if (is_acknowledged && (!enable || error == 0)) {
      slot->state.torque_enabled = enable;
    } else {
      failed.push_back(slot->state.id);
    }
  }
  return failed;
}

std::vector<LekiwiMotorDriverNode::MotorSlot *> LekiwiMotorDriverNode::MotorSlots(
  const std::vector<uint8_t> & ids)
{
  // ids 순서대로
  std::vector<MotorSlot *> motors;
  for (const uint8_t id : ids) {
    for (auto & slot : slots_) {
      if (slot.state.id == id) {
        motors.push_back(&slot);
      }
    }
  }
  return motors;
}

// ---------------------------------------------------------------------------
// 팔 쓰기
// ---------------------------------------------------------------------------

void LekiwiMotorDriverNode::CreateArmInterfaces()
{
  // 최신 목표만 의미가 있으므로 깊이 1
  sub_arm_command_ = create_subscription<lekiwi_msgs::msg::ArmCommand>(
    arm_command_topic_, rclcpp::QoS(1),
    [this](const lekiwi_msgs::msg::ArmCommand::SharedPtr msg) {ArmCommandCallback(msg);});
  srv_arm_torque_ = create_service<std_srvs::srv::SetBool>(
    arm_torque_service_,
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      ArmTorqueCallback(request, response);
    });
  // 나중에 붙은 구독자도 마지막 값을 받는다 (lekiwi/motor/ready와 같은 QoS)
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  pub_arm_ready_ = create_publisher<std_msgs::msg::Bool>(arm_ready_topic_, latched);
  pub_arm_registers_ = create_publisher<lekiwi_msgs::msg::ArmRegisters>(arm_registers_topic_, latched);
  pub_arm_ready_->publish(std_msgs::msg::Bool());   // false: 시작할 때는 팔 명령을 받지 않는다

  const std::size_t count = arm_ids_.size();
  arm_registers_.ids = arm_ids_;
  arm_registers_.valid.assign(count, false);
  arm_registers_.homing_offset.assign(count, 0);
  arm_registers_.min_position_limit.assign(count, 0);
  arm_registers_.max_position_limit.assign(count, 0);
  arm_registers_.p_coefficient.assign(count, 0);
  arm_registers_.d_coefficient.assign(count, 0);
  arm_registers_.i_coefficient.assign(count, 0);
  ReadArmRegisters();
  arm_registers_.header.stamp = now();
  pub_arm_registers_->publish(arm_registers_);

  RCLCPP_INFO(
    get_logger(),
    "arm write ready for ids [%s]: %s (Goal_Position ticks), %s (SetBool); "
    "arm torque is left as it is until requested",
    JoinIds(arm_ids_).c_str(), sub_arm_command_->get_topic_name(),
    srv_arm_torque_->get_service_name());
}

void LekiwiMotorDriverNode::ArmCommandCallback(const lekiwi_msgs::msg::ArmCommand::SharedPtr msg)
{
  if (!is_arm_ready_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "arm command ignored: arm is not ready (arm torque was not enabled with %s)",
      srv_arm_torque_->get_service_name());
    return;
  }
  if (msg->ids.empty() || msg->ids.size() != msg->positions_ticks.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "arm command ignored: ids and positions_ticks sizes differ");
    return;
  }
  std::set<uint8_t> seen;
  for (const uint8_t id : msg->ids) {
    // 바퀴 등 팔이 아닌 ID가 하나라도 있으면 메시지 전체를 버린다.
    if (std::find(arm_ids_.begin(), arm_ids_.end(), id) == arm_ids_.end() ||
      !seen.insert(id).second)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "arm command ignored: id %u is not a configured arm motor or is repeated", id);
      return;
    }
  }

  // 0~4095로 자른 뒤, 모터마다 마지막 Present_Position ± arm_max_goal_offset_ticks 안으로 자른다.
  // 잘못된 발행자가 있어도 한 명령으로 팔을 크게 움직이지 못한다.
  const std::vector<MotorSlot *> arms = MotorSlots(msg->ids);
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> entries;
  bool is_range_clamped = false;
  bool is_offset_clamped = false;
  for (std::size_t i = 0; i < arms.size(); ++i) {
    const auto & state = arms[i]->state;
    if (!state.online) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "arm command ignored: arm %u (%s) present position was not read this cycle",
        state.id, state.name.c_str());
      return;
    }
    const int ticks = std::clamp(msg->positions_ticks[i], 0, sts3215::kMaxPositionTicks);
    const int goal = std::clamp(
      ticks, state.position_ticks - arm_max_goal_offset_ticks_,
      state.position_ticks + arm_max_goal_offset_ticks_);
    is_range_clamped = is_range_clamped || ticks != msg->positions_ticks[i];
    is_offset_clamped = is_offset_clamped || goal != ticks;
    entries.emplace_back(state.id, sts3215::EncodeGoalPosition(goal));
  }
  if (is_range_clamped) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "arm command: positions outside [0, %d] were clamped",
      sts3215::kMaxPositionTicks);
  }
  if (is_offset_clamped) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "arm command: goals more than %d ticks from the present position were clamped",
      arm_max_goal_offset_ticks_);
  }

  // SYNC_WRITE는 응답이 없어 도착을 확인할 수 없다. 그래서 받은 명령마다 모든 목표를 한 패킷으로 쓴다.
  if (!bus_->SyncWrite(sts3215::kAddrGoalPosition, 2, entries)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "failed to write arm goal positions");
  }
}

void LekiwiMotorDriverNode::ArmTorqueCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::string message;
  response->success = request->data ? EnableArmTorque(message) : DisableArmTorque(message);
  response->message = message;
  if (response->success) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }
}

bool LekiwiMotorDriverNode::EnableArmTorque(std::string & message)
{
  const std::vector<MotorSlot *> arms = MotorSlots(arm_ids_);
  // 이미 명령을 받는 중이고 모든 팔 토크가 켜져 있으면 아무것도 쓰지 않는다 (잡고 있는 팔을 건드리지 않는다).
  if (is_arm_ready_ &&
    std::all_of(
      arms.begin(), arms.end(), [this](MotorSlot * slot) {
        return slot->responsive && ReadConfig(*slot) && ReadPresent(*slot) &&
        slot->state.torque_enabled;
      }))
  {
    // 보호 비트는 알리기만 하고 arm_ready는 그대로 둔다 (바뀐 것이 없다). 과부하는 빼고 본다:
    // 물건을 쥔 그리퍼에 서고, 막힌 관절의 과부하는 팔로워가 멈춘다.
    for (const MotorSlot * slot : arms) {
      const uint8_t fault = slot->state.status & sts3215::kStatusFaultMask &
        static_cast<uint8_t>(~sts3215::kStatusOverload);
      if (fault != 0) {
        message = "arm torque already enabled, but arm " + std::to_string(slot->state.id) + " (" +
          slot->state.name + ") reports " + sts3215::StatusToString(fault) + "; nothing written";
        return false;
      }
    }
    message = "arm torque already enabled; nothing written";
    return true;
  }

  // 이 호출이 성공할 때까지 팔 명령을 받지 않는다.
  SetArmReady(false);
  if (!arm_pending_torque_off_.empty()) {
    message = "refused: arm torque disable of ids [" + JoinIds(arm_pending_torque_off_) +
      "] is not confirmed yet";
    return false;
  }

  // 켜기 전에 모든 팔이 응답하고, 위치 모드이고, 보호 비트가 꺼져 있는지 지금 다시 읽는다.
  for (MotorSlot * slot : arms) {
    auto & state = slot->state;
    const std::string arm = "arm " + std::to_string(state.id) + " (" + state.name + ")";
    if (!slot->responsive || !ReadConfig(*slot) || !ReadPresent(*slot)) {
      message = "refused: " + arm + " is not responding";
      return false;
    }
    if (state.mode != sts3215::kModePosition) {
      message = "refused: " + arm + " is in mode " + std::to_string(state.mode) +
        ", not position mode (0)";
      return false;
    }
    // 이미 토크가 켜진 팔의 과부하는 보지 않는다: 쓰지 않을 팔이고, 물건을 쥔 그리퍼에 선다
    const uint8_t fault_mask = state.torque_enabled ?
      static_cast<uint8_t>(sts3215::kStatusFaultMask & ~sts3215::kStatusOverload) :
      sts3215::kStatusFaultMask;
    if ((state.status & fault_mask) != 0) {
      message = "refused: " + arm + " reports " + sts3215::StatusToString(state.status & fault_mask);
      return false;
    }
    if (state.position_ticks < 0 || state.position_ticks > sts3215::kMaxPositionTicks) {
      message = "refused: " + arm + " present position " + std::to_string(state.position_ticks) +
        " is outside [0, 4095]";
      return false;
    }
  }

  // 방금 읽은 토크 상태를 기억한다. 꺼져 있던 팔만 켜고, 실패하면 그 팔만 다시 끈다.
  // 잡고 있던 팔은 목표를 다시 쓰지 않는다 (지금 위치로 다시 쓰면 무게에 눌린 만큼 처진다).
  std::vector<uint8_t> off_before;
  for (const MotorSlot * slot : arms) {
    if (!slot->state.torque_enabled) {
      off_before.push_back(slot->state.id);
    }
  }
  std::vector<uint8_t> failed = WriteArmGoalAndTorque(MotorSlots(off_before));
  if (!failed.empty()) {
    failed = WriteArmGoalAndTorque(MotorSlots(failed));   // 실패한 팔만 한 번 더
  }
  if (!failed.empty()) {
    const std::vector<uint8_t> not_disabled = WriteTorqueIds(off_before, false);
    arm_pending_torque_off_.insert(not_disabled.begin(), not_disabled.end());
    message = "failed to enable arm torque: no acknowledgement from arm ids [" + JoinIds(failed) +
      "]; torque turned back off on ids [" + JoinIds(off_before) + "] (off before this call)";
    if (!not_disabled.empty()) {
      message += "; retrying torque disable of ids [" + JoinIds(not_disabled) + "] every slow cycle";
    }
    return false;
  }

  SetArmReady(true);
  message = off_before.empty() ?
    "arm commands accepted; arm torque was already on (nothing written)" :
    "arm torque enabled on ids [" + JoinIds(off_before) + "] (goal = present position)";
  return true;
}

std::vector<uint8_t> LekiwiMotorDriverNode::WriteArmGoalAndTorque(
  const std::vector<MotorSlot *> & arms)
{
  // 목표 레지스터를 지금 위치로 맞춘 팔만 토크를 켠다. 남아 있던 옛 목표로 튀지 않게 한다.
  std::vector<uint8_t> failed;
  std::vector<uint8_t> goal_set;
  for (const MotorSlot * slot : arms) {
    uint8_t error = 0;
    const bool is_set = bus_->Write(
      slot->state.id, sts3215::kAddrGoalPosition,
      sts3215::EncodeGoalPosition(slot->state.position_ticks), error) && error == 0;
    (is_set ? goal_set : failed).push_back(slot->state.id);
  }
  const std::vector<uint8_t> torque_failed = WriteTorqueIds(goal_set, true);
  failed.insert(failed.end(), torque_failed.begin(), torque_failed.end());
  std::sort(failed.begin(), failed.end());
  return failed;
}

bool LekiwiMotorDriverNode::DisableArmTorque(std::string & message)
{
  // 명령을 먼저 막고, 팔 ID에만 하나씩 끈다 (브로드캐스트 · 바퀴 없음). 하나가 실패해도 나머지는 보낸다.
  SetArmReady(false);
  const std::vector<uint8_t> failed = WriteTorqueIds(arm_ids_, false);
  arm_pending_torque_off_ = std::set<uint8_t>(failed.begin(), failed.end());
  if (!failed.empty()) {
    message = "failed to disable arm torque: no acknowledgement from arm ids [" + JoinIds(failed) +
      "]; retrying them every slow cycle";
    return false;
  }
  message = "arm torque disabled";
  return true;
}

void LekiwiMotorDriverNode::RetryArmTorqueOff()
{
  if (arm_pending_torque_off_.empty()) {
    return;
  }
  // 확인하지 못한 팔에만 하나씩 다시 보낸다 (브로드캐스트 없음)
  const std::vector<uint8_t> pending(arm_pending_torque_off_.begin(), arm_pending_torque_off_.end());
  const std::vector<uint8_t> failed = WriteTorqueIds(pending, false);
  arm_pending_torque_off_ = std::set<uint8_t>(failed.begin(), failed.end());
  if (failed.empty()) {
    RCLCPP_INFO(get_logger(), "arm torque disable confirmed for ids [%s]", JoinIds(pending).c_str());
  }
}

void LekiwiMotorDriverNode::SetArmReady(const bool is_ready)
{
  if (is_ready == is_arm_ready_) {
    return;
  }
  is_arm_ready_ = is_ready;
  std_msgs::msg::Bool message;
  message.data = is_ready;
  pub_arm_ready_->publish(message);
}

bool LekiwiMotorDriverNode::ReadArmRegisters()
{
  // EEPROM은 읽기만 한다. 아직 읽지 못했고 지금 응답하는 팔만 읽는다. 하나라도 새로 읽으면 true.
  bool is_updated = false;
  const std::vector<MotorSlot *> arms = MotorSlots(arm_ids_);
  for (std::size_t i = 0; i < arms.size(); ++i) {
    std::vector<uint8_t> data;
    uint8_t error = 0;
    if (arm_registers_.valid[i] || !arms[i]->responsive ||
      !bus_->Read(
        arms[i]->state.id, sts3215::kEepromBlockAddress, sts3215::kEepromBlockLength, data, error))
    {
      continue;
    }
    const auto at = [&data](const uint8_t address) {
        return data.data() + (address - sts3215::kEepromBlockAddress);
      };
    arm_registers_.valid[i] = true;
    arm_registers_.min_position_limit[i] = sts3215::Word(at(sts3215::kAddrMinPositionLimit));
    arm_registers_.max_position_limit[i] = sts3215::Word(at(sts3215::kAddrMaxPositionLimit));
    arm_registers_.p_coefficient[i] = *at(sts3215::kAddrPCoefficient);
    arm_registers_.d_coefficient[i] = *at(sts3215::kAddrDCoefficient);
    arm_registers_.i_coefficient[i] = *at(sts3215::kAddrICoefficient);
    // LeRobot 인코딩과 같다: Homing_Offset은 부호-크기, 부호 비트 11
    arm_registers_.homing_offset[i] =
      sts3215::DecodeSignMagnitude(sts3215::Word(at(sts3215::kAddrHomingOffset)), 11);
    is_updated = true;
  }
  return is_updated;
}

}  // namespace lekiwi_motor_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<lekiwi_motor_driver::LekiwiMotorDriverNode>();
    rclcpp::spin(node);
    node.reset();   // 종료 처리(바퀴 토크 끄기)를 shutdown 전에 끝낸다
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lekiwi_motor_driver_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
