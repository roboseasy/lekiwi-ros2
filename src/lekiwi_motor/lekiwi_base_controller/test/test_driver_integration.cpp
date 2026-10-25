// 가짜 모터 드라이버와 함께 도는 ROS 통합 테스트.
// 실제 드라이버 · 로봇과 섞이지 않도록 프로세스마다 다른 namespace를 쓰고,
// CMake에서 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST로 실행한다.
#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <lekiwi_msgs/msg/motor_state.hpp>
#include <lekiwi_msgs/msg/motor_state_array.hpp>
#include <lekiwi_msgs/msg/wheel_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "lekiwi_base_controller/driver_motor_interface.hpp"
#include "lekiwi_base_controller/lekiwi_base_controller_node.hpp"
#include "lekiwi_base_controller/motor_safety_coordinator.hpp"
#include "lekiwi_base_controller/omni_kinematics.hpp"

namespace
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using lekiwi_msgs::msg::MotorState;
using lekiwi_msgs::msg::MotorStateArray;
using lekiwi_msgs::msg::WheelCommand;
using std_srvs::srv::SetBool;

double SecondsSince(const SteadyClock::time_point start)
{
  return std::chrono::duration<double>(SteadyClock::now() - start).count();
}

template<typename Predicate>
bool WaitFor(Predicate predicate, const std::chrono::milliseconds timeout)
{
  const auto deadline = SteadyClock::now() + timeout;
  while (SteadyClock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

rclcpp::NodeOptions NamespacedOptions(const std::string & name_space)
{
  return rclcpp::NodeOptions()
         .use_global_arguments(false)
         .arguments({"--ros-args", "-r", "__ns:=" + name_space});
}

// lekiwi_motor_driver_node 대역: states 발행, wheel_torque 서비스, wheel_command 기록
class FakeMotorDriver : public rclcpp::Node
{
public:
  struct CommandRecord
  {
    SteadyClock::time_point time;
    WheelCommand message;
  };

  FakeMotorDriver(const rclcpp::NodeOptions & options, const bool is_auto_publishing)
  : Node("fake_motor_driver", options)
  {
    pub_states_ = create_publisher<MotorStateArray>("lekiwi/motor/states", rclcpp::SensorDataQoS());
    sub_command_ = create_subscription<WheelCommand>(
      "lekiwi/motor/wheel_command", rclcpp::QoS(1),
      [this](const WheelCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_.push_back(CommandRecord{SteadyClock::now(), *msg});
      });
    srv_torque_ = create_service<SetBool>(
      "lekiwi/motor/wheel_torque",
      [this](
        const std::shared_ptr<SetBool::Request> request,
        std::shared_ptr<SetBool::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        torque_requests_.push_back(request->data);
        response->success = !is_torque_failing;
        if (response->success) {
          is_torque_on_ = request->data;
        }
        response->message = request->data ? "fake torque on" : "fake torque off";
      });
    if (is_auto_publishing) {
      timer_ = create_wall_timer(50ms, [this]() {
            if (is_publishing) {
              PublishStates();
            }
          });
    }
  }

  // stamp · 바퀴 torque_enabled를 지정하면 그 값으로 한 번 발행한다 (늦게 도착한 메시지 흉내)
  void PublishStates(
    const std::optional<rclcpp::Time> & stamp = std::nullopt,
    const std::optional<bool> & reported_torque = std::nullopt)
  {
    MotorStateArray msg;
    msg.header.stamp = stamp ? *stamp : now();
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint8_t id = 1; id <= 9; ++id) {
      MotorState motor;
      motor.id = id;
      motor.online = true;
      motor.position_ticks = 1000 + id;
      motor.mode = MotorState::MODE_VELOCITY;
      motor.torque_enabled = id >= 7 && reported_torque.value_or(is_torque_on_);
      auto offline = offline_remaining_.find(id);
      if (offline != offline_remaining_.end() && offline->second != 0) {
        motor.online = false;
        motor.position_ticks = 0;
        if (offline->second > 0) {
          --offline->second;
        }
      }
      msg.motors.push_back(motor);
    }
    pub_states_->publish(msg);
  }

  // 드라이버가 서비스 요청 없이 스스로 토크를 켜거나 끈 상황 (watchdog · 모터 보호)
  void SetTorqueOn(const bool is_on)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_torque_on_ = is_on;
  }

  // count개 메시지 동안 offline. count < 0이면 계속
  void SetOffline(const uint8_t id, const int count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    offline_remaining_[id] = count;
  }

  bool HasOfflinePending(const uint8_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto offline = offline_remaining_.find(id);
    return offline != offline_remaining_.end() && offline->second != 0;
  }

  std::vector<bool> TorqueRequests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return torque_requests_;
  }

  void ClearTorqueRequests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    torque_requests_.clear();
  }

  std::vector<CommandRecord> Commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }

  void ClearCommands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
  }

  bool IsWired() const
  {
    return pub_states_->get_subscription_count() > 0 &&
           sub_command_->get_publisher_count() > 0;
  }

  std::atomic<bool> is_publishing{true};
  std::atomic<bool> is_torque_failing{false};

private:
  rclcpp::Publisher<MotorStateArray>::SharedPtr pub_states_;
  rclcpp::Subscription<WheelCommand>::SharedPtr sub_command_;
  rclcpp::Service<SetBool>::SharedPtr srv_torque_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::vector<bool> torque_requests_;
  std::vector<CommandRecord> commands_;
  std::map<uint8_t, int> offline_remaining_;
  bool is_torque_on_{false};
};

// 조작 창 대역: cmd_vel 발행, power 서비스 호출, ready 구독
class TestOperator : public rclcpp::Node
{
public:
  explicit TestOperator(const rclcpp::NodeOptions & options)
  : Node("test_operator", options)
  {
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
    power_client_ = create_client<SetBool>("lekiwi/motor/power");
    sub_ready_ = create_subscription<std_msgs::msg::Bool>(
      "lekiwi/motor/ready", lekiwi_base_controller::MakeMotorReadyQos(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_history_.emplace_back(SteadyClock::now(), msg->data);
      });
  }

  std::optional<SetBool::Response> CallPower(
    const bool is_on, double * elapsed_seconds, const std::chrono::milliseconds timeout = 3000ms)
  {
    if (!power_client_->wait_for_service(5s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<SetBool::Request>();
    request->data = is_on;
    const auto start = SteadyClock::now();
    auto future = power_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      power_client_->remove_pending_request(future);
      return std::nullopt;
    }
    if (elapsed_seconds != nullptr) {
      *elapsed_seconds = SecondsSince(start);
    }
    return *future.get();
  }

  void PublishCmdVel(const double linear_x)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    pub_cmd_vel_->publish(twist);
  }

  std::optional<bool> LatestReady()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ready_history_.empty()) {
      return std::nullopt;
    }
    return ready_history_.back().second;
  }

  // start 이후 처음 value가 된 시각
  std::optional<SteadyClock::time_point> FirstReadyAfter(
    const SteadyClock::time_point start, const bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [time, data] : ready_history_) {
      if (time >= start && data == value) {
        return time;
      }
    }
    return std::nullopt;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Client<SetBool>::SharedPtr power_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ready_;
  std::mutex mutex_;
  std::vector<std::pair<SteadyClock::time_point, bool>> ready_history_;
};

bool IsZero(const WheelCommand & msg)
{
  return std::all_of(
    msg.velocity.begin(), msg.velocity.end(), [](const double v) {return v == 0.0;});
}

std::string UniqueNamespace(const char * prefix)
{
  static std::atomic<int> counter{0};
  return std::string("/") + prefix + "_" + std::to_string(::getpid()) + "_" +
         std::to_string(counter++);
}

// 실행기 하나를 백그라운드 스레드에서 돌린다
class SpinningExecutor
{
public:
  explicit SpinningExecutor(const std::size_t thread_count)
  : executor_(rclcpp::ExecutorOptions(), thread_count)
  {
  }

  ~SpinningExecutor() {Stop();}

  void Add(const rclcpp::Node::SharedPtr & node)
  {
    executor_.add_node(node);
    nodes_.push_back(node);
  }

  void Start()
  {
    thread_ = std::thread([this]() {executor_.spin();});
  }

  void Stop()
  {
    if (thread_.joinable()) {
      executor_.cancel();
      thread_.join();
    }
    for (const auto & node : nodes_) {
      executor_.remove_node(node);
    }
    nodes_.clear();
  }

private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread thread_;
  std::vector<rclcpp::Node::SharedPtr> nodes_;
};

}  // namespace

/*********************************************************
  노드 전체: 가짜 드라이버 + 베이스 컨트롤러 + 조작 창
 *********************************************************/

class BaseControllerDriverTest : public ::testing::Test
{
protected:
  static constexpr double cmd_vel_timeout = 0.3;
  static constexpr double driver_states_timeout = 0.5;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    name_space_ = UniqueNamespace("base_controller_test");
    driver_ = std::make_shared<FakeMotorDriver>(NamespacedOptions(name_space_), true);
    operator_ = std::make_shared<TestOperator>(NamespacedOptions(name_space_));
    helper_executor_ = std::make_unique<SpinningExecutor>(2);
    helper_executor_->Add(driver_);
    helper_executor_->Add(operator_);
    helper_executor_->Start();

    auto options = NamespacedOptions(name_space_);
    options.parameter_overrides({
        {"motor_backend", "driver"},
        {"enable_motor_write", true},
        {"odom_source", "encoder"},
        {"encoder_feedback_source", "position"},
        {"wheel_directions", wheel_directions_},
        {"torque_enable", false}});
    controller_ = std::make_shared<lekiwi_base_controller::LekiwiBaseControllerNode>(options);
    // 실제 main과 같은 실행기 구성
    controller_executor_ = std::make_unique<SpinningExecutor>(
      lekiwi_base_controller::LekiwiBaseControllerNode::executor_thread_count);
    controller_executor_->Add(controller_);
    controller_executor_->Start();

    ASSERT_TRUE(WaitFor([this]() {return driver_->IsWired();}, 5000ms));
    ASSERT_TRUE(WaitFor([this]() {return operator_->LatestReady().has_value();}, 5000ms));
    EXPECT_FALSE(*operator_->LatestReady());
    // states가 몇 번 오가도록 기다린다
    std::this_thread::sleep_for(300ms);
  }

  void TearDown() override
  {
    controller_executor_.reset();
    helper_executor_.reset();
    controller_.reset();
    operator_.reset();
    driver_.reset();
    rclcpp::shutdown();
  }

  void PowerOnAndWaitReady()
  {
    double elapsed = 0.0;
    const auto response = operator_->CallPower(true, &elapsed);
    ASSERT_TRUE(response.has_value()) << "power service did not answer (deadlock?)";
    ASSERT_TRUE(response->success) << response->message;
    // 중첩 토크 서비스 호출이 막히지 않았다면 timeout(1 s)보다 훨씬 빨리 끝난다
    EXPECT_LT(elapsed, 0.9);
    ASSERT_TRUE(WaitFor([this]() {return operator_->LatestReady() == true;}, 1500ms));
  }

  std::vector<double> wheel_directions_{1.0, -1.0, 1.0};
  std::string name_space_;
  std::shared_ptr<FakeMotorDriver> driver_;
  std::shared_ptr<TestOperator> operator_;
  std::shared_ptr<lekiwi_base_controller::LekiwiBaseControllerNode> controller_;
  std::unique_ptr<SpinningExecutor> helper_executor_;
  std::unique_ptr<SpinningExecutor> controller_executor_;
};

TEST_F(BaseControllerDriverTest, PowerDriveTimeoutFaultAndRecoveryFollowTheSafetyContract)
{
  // (1) power true → 토크 켜기 요청 → 새 샘플 후 ready true
  PowerOnAndWaitReady();
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{true}));

  // (2) cmd_vel → 0이 아닌 WheelCommand (ids 모델 순서, 방향 부호 적용)
  driver_->ClearCommands();
  constexpr double kLinearX = 0.05;
  for (int i = 0; i < 10; ++i) {
    operator_->PublishCmdVel(kLinearX);
    std::this_thread::sleep_for(50ms);
  }
  const auto last_cmd_vel_time = SteadyClock::now();
  const lekiwi_base_controller::BaseControllerParameters defaults;
  const lekiwi_base_controller::OmniKinematics kinematics(
    defaults.wheel_radius, defaults.base_radius, defaults.max_wheel_speed,
    defaults.kinematics_frame_x, defaults.kinematics_frame_y, defaults.kinematics_frame_yaw);
  const auto expected_model = kinematics.ToWheelSpeeds(kLinearX, 0.0, 0.0);
  const std::vector<double> expected{
    expected_model.left * wheel_directions_[0],
    expected_model.back * wheel_directions_[1],
    expected_model.right * wheel_directions_[2]};
  ASSERT_GT(std::abs(expected[1]), 0.1);
  ASSERT_GT(std::abs(expected[2]), 0.1);

  const auto drive_commands = driver_->Commands();
  const auto moving = std::find_if(
    drive_commands.begin(), drive_commands.end(),
    [](const FakeMotorDriver::CommandRecord & record) {return !IsZero(record.message);});
  ASSERT_NE(moving, drive_commands.end()) << "no non-zero wheel command";
  EXPECT_EQ(moving->message.ids, (std::vector<uint8_t>{8, 9, 7}));
  ASSERT_EQ(moving->message.velocity.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(moving->message.velocity[i], expected[i], 1e-9) << i;
  }

  // (3) cmd_vel을 멈추면 cmd_vel_timeout 안에 속도 0
  std::this_thread::sleep_for(700ms);
  const auto stop_commands = driver_->Commands();
  std::optional<SteadyClock::time_point> first_zero;
  for (const auto & record : stop_commands) {
    if (record.time < last_cmd_vel_time) {
      continue;
    }
    if (IsZero(record.message)) {
      if (!first_zero) {
        first_zero = record.time;
      }
    } else {
      EXPECT_FALSE(first_zero.has_value()) << "non-zero command after the stop";
    }
  }
  ASSERT_TRUE(first_zero.has_value());
  EXPECT_LE(
    std::chrono::duration<double>(*first_zero - last_cmd_vel_time).count(),
    cmd_vel_timeout + 0.15);
  EXPECT_EQ(operator_->LatestReady(), true);

  // (4) states가 끊기면 ready false, 토크 끄기 요청, 고장 고정
  driver_->ClearTorqueRequests();
  driver_->ClearCommands();
  const auto loss_time = SteadyClock::now();
  driver_->is_publishing = false;
  ASSERT_TRUE(WaitFor([this]() {return operator_->LatestReady() == false;}, 2000ms));
  const auto ready_false_time = operator_->FirstReadyAfter(loss_time, false);
  ASSERT_TRUE(ready_false_time.has_value());
  EXPECT_LE(
    std::chrono::duration<double>(*ready_false_time - loss_time).count(),
    driver_states_timeout + 0.25);
  ASSERT_TRUE(
    WaitFor(
      [this]() {
        const auto requests = driver_->TorqueRequests();
        return std::find(requests.begin(), requests.end(), false) != requests.end();
      }, 1500ms));
  for (const bool request : driver_->TorqueRequests()) {
    EXPECT_FALSE(request);
  }
  for (const auto & record : driver_->Commands()) {
    EXPECT_TRUE(IsZero(record.message)) << "wheel command after link loss must be zero";
  }

  // (5) 드라이버가 돌아와도 power false 전에는 power true를 거절한다
  driver_->is_publishing = true;
  std::this_thread::sleep_for(300ms);
  driver_->ClearTorqueRequests();
  double elapsed = 0.0;
  auto response = operator_->CallPower(true, &elapsed);
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("disable motor power"), std::string::npos) << response->message;
  EXPECT_TRUE(driver_->TorqueRequests().empty());
  EXPECT_EQ(operator_->LatestReady(), false);

  // (6) power false → power true 순서로만 복구된다
  response = operator_->CallPower(false, &elapsed);
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_LT(elapsed, 0.9);
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{false}));
  PowerOnAndWaitReady();
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{false, true}));
}

TEST_F(BaseControllerDriverTest, BriefWheelOfflineDoesNotFaultButLongOfflineDoes)
{
  PowerOnAndWaitReady();
  driver_->ClearTorqueRequests();
  const auto start = SteadyClock::now();

  // 메시지 하나만 offline (드라이버 한 주기 읽기 실패)
  driver_->SetOffline(9, 1);
  ASSERT_TRUE(WaitFor([this]() {return !driver_->HasOfflinePending(9);}, 1000ms));
  std::this_thread::sleep_for(300ms);
  // 연속 5개 (0.25 s) offline: 엔코더 읽기 실패가 반드시 생기지만 마감 시간 안이다
  driver_->SetOffline(9, 5);
  ASSERT_TRUE(WaitFor([this]() {return !driver_->HasOfflinePending(9);}, 1000ms));
  std::this_thread::sleep_for(600ms);

  EXPECT_FALSE(operator_->FirstReadyAfter(start, false).has_value()) << "brief offline faulted";
  EXPECT_EQ(operator_->LatestReady(), true);
  EXPECT_TRUE(driver_->TorqueRequests().empty());

  // driver_states_timeout보다 오래 offline이면 고장
  const auto offline_time = SteadyClock::now();
  driver_->SetOffline(9, -1);
  ASSERT_TRUE(WaitFor([this]() {return operator_->LatestReady() == false;}, 2000ms));
  const auto ready_false_time = operator_->FirstReadyAfter(offline_time, false);
  ASSERT_TRUE(ready_false_time.has_value());
  const double fault_delay =
    std::chrono::duration<double>(*ready_false_time - offline_time).count();
  EXPECT_GE(fault_delay, 0.3);
  EXPECT_LE(fault_delay, driver_states_timeout + 0.25);
  // 연결 끊김 처리와 엔코더 시간 초과가 같은 주기에 겹치면 끄기 요청이 두 번 갈 수 있다 (무해)
  ASSERT_TRUE(WaitFor([this]() {return !driver_->TorqueRequests().empty();}, 1500ms));
  for (const bool request : driver_->TorqueRequests()) {
    EXPECT_FALSE(request);
  }

  // 바퀴가 돌아와도 고장은 유지된다
  driver_->SetOffline(9, 0);
  std::this_thread::sleep_for(300ms);
  double elapsed = 0.0;
  const auto response = operator_->CallPower(true, &elapsed);
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
}

TEST_F(BaseControllerDriverTest, DriverRefusingTorqueKeepsMotorNotReady)
{
  driver_->is_torque_failing = true;
  double elapsed = 0.0;
  const auto response = operator_->CallPower(true, &elapsed);

  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  EXPECT_LT(elapsed, 0.9);
  // 켜기 실패 → 속도 0 → 끄기 시도
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{true, false}));
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(operator_->LatestReady(), false);
  for (const auto & record : driver_->Commands()) {
    EXPECT_TRUE(IsZero(record.message));
  }
}

TEST_F(BaseControllerDriverTest, DriverTorqueOffWhileReadyLatchesFaultButStaleMessageDoesNot)
{
  const rclcpp::Time before_enable = driver_->now();
  PowerOnAndWaitReady();
  driver_->ClearTorqueRequests();
  driver_->ClearCommands();

  // 켜기 전에 찍힌 torque_enabled=false 메시지가 늦게 도착해도 고장이 아니다
  const auto stale_time = SteadyClock::now();
  driver_->PublishStates(before_enable, false);
  std::this_thread::sleep_for(500ms);
  EXPECT_FALSE(operator_->FirstReadyAfter(stale_time, false).has_value()) << "stale message faulted";
  EXPECT_EQ(operator_->LatestReady(), true);
  EXPECT_TRUE(driver_->TorqueRequests().empty());

  // 드라이버가 스스로 토크를 끄면 다음 states에서 고장 고정, ready false, 토크 끔 요청
  const auto off_time = SteadyClock::now();
  driver_->SetTorqueOn(false);
  ASSERT_TRUE(WaitFor([this]() {return operator_->LatestReady() == false;}, 1000ms));
  const auto ready_false_time = operator_->FirstReadyAfter(off_time, false);
  ASSERT_TRUE(ready_false_time.has_value());
  // states 한 주기(50 ms) + 제어 한 주기(20 ms) 안팎. driver_states_timeout을 기다리지 않는다
  EXPECT_LE(std::chrono::duration<double>(*ready_false_time - off_time).count(), 0.25);
  ASSERT_TRUE(WaitFor([this]() {return !driver_->TorqueRequests().empty();}, 1000ms));
  for (const bool request : driver_->TorqueRequests()) {
    EXPECT_FALSE(request);
  }
  for (const auto & record : driver_->Commands()) {
    if (record.time >= off_time) {
      EXPECT_TRUE(IsZero(record.message));
    }
  }

  // power false 전에는 다시 켜지 않고, false → true로 복구된다
  double elapsed = 0.0;
  auto response = operator_->CallPower(true, &elapsed);
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  response = operator_->CallPower(false, &elapsed);
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success) << response->message;
  PowerOnAndWaitReady();
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(operator_->LatestReady(), true);
}

/*********************************************************
  드라이버 없이: 서비스가 막히지 않고 시간 안에 실패한다
 *********************************************************/

TEST(BaseControllerWithoutDriverTest, PowerRequestFailsWithinTimeoutWithoutDriver)
{
  rclcpp::init(0, nullptr);
  {
    const auto name_space = UniqueNamespace("base_controller_alone");
    auto operator_node = std::make_shared<TestOperator>(NamespacedOptions(name_space));
    SpinningExecutor helper(2);
    helper.Add(operator_node);
    helper.Start();

    auto options = NamespacedOptions(name_space);
    options.parameter_overrides({{"motor_backend", "driver"}, {"enable_motor_write", true}});
    auto controller = std::make_shared<lekiwi_base_controller::LekiwiBaseControllerNode>(options);
    SpinningExecutor controller_executor(
      lekiwi_base_controller::LekiwiBaseControllerNode::executor_thread_count);
    controller_executor.Add(controller);
    controller_executor.Start();

    double elapsed = 0.0;
    const auto response = operator_node->CallPower(true, &elapsed, 4000ms);
    ASSERT_TRUE(response.has_value()) << "power service blocked";
    EXPECT_FALSE(response->success);
    // states가 없으므로 켜기를 시도하지 않고, 끄기 호출 하나가 torque_service_timeout(1 s)에 끝난다
    EXPECT_LT(elapsed, 1.0 + 0.5);
    EXPECT_EQ(operator_node->LatestReady(), false);

    controller_executor.Stop();
    helper.Stop();
  }
  rclcpp::shutdown();
}

TEST(BaseControllerStartupTest, TorqueEnableParameterIsRejected)
{
  rclcpp::init(0, nullptr);
  {
    auto options = NamespacedOptions(UniqueNamespace("base_controller_torque"));
    options.parameter_overrides({{"torque_enable", true}});
    EXPECT_THROW(
      std::make_shared<lekiwi_base_controller::LekiwiBaseControllerNode>(options),
      std::invalid_argument);
  }
  rclcpp::shutdown();
}

/*********************************************************
  DriverMotorInterface 단독: 바퀴별 연결 판단 · 새 메시지만 사용
 *********************************************************/

class DriverMotorInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    const auto name_space = UniqueNamespace("driver_interface_test");
    driver_ = std::make_shared<FakeMotorDriver>(NamespacedOptions(name_space), false);
    host_ = std::make_shared<rclcpp::Node>("driver_interface_host", NamespacedOptions(name_space));
    lekiwi_base_controller::DriverMotorConfig config;
    config.feedback.source = lekiwi_base_controller::EncoderFeedbackSource::Speed;
    interface_ = std::make_unique<lekiwi_base_controller::DriverMotorInterface>(*host_, config);
    executor_ = std::make_unique<SpinningExecutor>(3);
    executor_->Add(driver_);
    executor_->Add(host_);
    executor_->Start();
    ASSERT_TRUE(WaitFor([this]() {return driver_->IsWired();}, 5000ms));
  }

  void TearDown() override
  {
    executor_.reset();
    interface_.reset();
    host_.reset();
    driver_.reset();
    rclcpp::shutdown();
  }

  // 메시지를 하나 보내고 구독 콜백이 받을 시간을 준다
  void PublishAndSettle()
  {
    driver_->PublishStates();
    std::this_thread::sleep_for(60ms);
  }

  std::shared_ptr<FakeMotorDriver> driver_;
  rclcpp::Node::SharedPtr host_;
  std::unique_ptr<lekiwi_base_controller::DriverMotorInterface> interface_;
  std::unique_ptr<SpinningExecutor> executor_;
};

TEST_F(DriverMotorInterfaceTest, SingleOfflineMessageIsAFailedReadNotADisconnect)
{
  EXPECT_FALSE(interface_->IsConnected());
  EXPECT_FALSE(interface_->ReadWheelFeedback().has_value());

  PublishAndSettle();
  EXPECT_TRUE(interface_->IsConnected());
  EXPECT_TRUE(interface_->ReadWheelFeedback().has_value());
  // 같은 메시지는 다시 쓰지 않는다
  EXPECT_FALSE(interface_->ReadWheelFeedback().has_value());

  driver_->SetOffline(9, 1);
  PublishAndSettle();
  EXPECT_TRUE(interface_->IsConnected());
  EXPECT_FALSE(interface_->ReadWheelFeedback().has_value());

  PublishAndSettle();
  EXPECT_TRUE(interface_->IsConnected());
  EXPECT_TRUE(interface_->ReadWheelFeedback().has_value());
}

TEST_F(DriverMotorInterfaceTest, WheelOfflineLongerThanTimeoutOrMissingStatesDisconnects)
{
  PublishAndSettle();
  ASSERT_TRUE(interface_->IsConnected());

  // 다른 바퀴는 계속 오지만 ID 7이 0.5 s 넘게 offline
  driver_->SetOffline(7, -1);
  const auto offline_start = SteadyClock::now();
  bool is_disconnected = false;
  while (SecondsSince(offline_start) < 1.0) {
    driver_->PublishStates();
    std::this_thread::sleep_for(50ms);
    if (!interface_->IsConnected()) {
      is_disconnected = true;
      break;
    }
  }
  ASSERT_TRUE(is_disconnected);
  EXPECT_GE(SecondsSince(offline_start), 0.45);

  // 다시 online이면 바로 연결됨
  driver_->SetOffline(7, 0);
  PublishAndSettle();
  EXPECT_TRUE(interface_->IsConnected());

  // states가 끊기면 driver_states_timeout 뒤 연결 끊김
  std::this_thread::sleep_for(300ms);
  EXPECT_TRUE(interface_->IsConnected());
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(interface_->IsConnected());
}

TEST_F(DriverMotorInterfaceTest, PowerControlsTorqueAndGatesWheelCommands)
{
  PublishAndSettle();
  EXPECT_FALSE(interface_->IsMotorPowerEnabled());

  // 전원이 꺼져 있으면 명령을 보내지 않는다
  interface_->WriteWheelSpeeds({1.0, 1.0, 1.0});
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(driver_->Commands().empty());

  ASSERT_TRUE(interface_->SetMotorPower(true));
  EXPECT_TRUE(interface_->IsMotorPowerEnabled());
  // max_wheel_speed로 잘린다
  interface_->WriteWheelSpeeds({9.0, -0.5, 0.25});
  ASSERT_TRUE(WaitFor([this]() {return !driver_->Commands().empty();}, 1000ms));
  const auto command = driver_->Commands().back().message;
  EXPECT_EQ(command.ids, (std::vector<uint8_t>{8, 9, 7}));
  EXPECT_DOUBLE_EQ(command.velocity[0], 2.3009711818284617);
  EXPECT_DOUBLE_EQ(command.velocity[1], -0.5);
  EXPECT_DOUBLE_EQ(command.velocity[2], 0.25);

  // 끄기: 속도 0을 먼저 보내고 토크를 끈다
  driver_->ClearCommands();
  ASSERT_TRUE(interface_->SetMotorPower(false));
  EXPECT_FALSE(interface_->IsMotorPowerEnabled());
  ASSERT_TRUE(WaitFor([this]() {return !driver_->Commands().empty();}, 1000ms));
  EXPECT_TRUE(IsZero(driver_->Commands().front().message));
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{true, false}));
}

TEST_F(DriverMotorInterfaceTest, LinkLossWhilePoweredWritesZeroAndDisablesTorque)
{
  PublishAndSettle();
  ASSERT_TRUE(interface_->SetMotorPower(true));
  driver_->ClearTorqueRequests();
  driver_->ClearCommands();

  // states가 끊긴 채 driver_states_timeout이 지나면 다음 쓰기에서 속도 0 → 토크 끔
  std::this_thread::sleep_for(600ms);
  EXPECT_FALSE(interface_->IsConnected());
  interface_->WriteWheelSpeeds({1.0, 1.0, 1.0});

  EXPECT_FALSE(interface_->IsMotorPowerEnabled());
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{false}));
  ASSERT_TRUE(WaitFor([this]() {return !driver_->Commands().empty();}, 1000ms));
  std::this_thread::sleep_for(60ms);
  for (const auto & record : driver_->Commands()) {
    EXPECT_TRUE(IsZero(record.message));
  }
}

TEST_F(DriverMotorInterfaceTest, UnconfirmedTorqueOffStopsPublishingCommands)
{
  PublishAndSettle();
  ASSERT_TRUE(interface_->SetMotorPower(true));
  driver_->is_torque_failing = true;
  driver_->ClearCommands();

  EXPECT_FALSE(interface_->SetMotorPower(false));
  ASSERT_TRUE(WaitFor([this]() {return !driver_->Commands().empty();}, 1000ms));
  std::this_thread::sleep_for(60ms);
  driver_->ClearCommands();
  // 끄기를 확인하지 못했으므로 이후 명령은 보내지 않는다 (드라이버 watchdog에 맡김)
  interface_->WriteWheelSpeeds({1.0, 1.0, 1.0});
  std::this_thread::sleep_for(100ms);
  EXPECT_TRUE(driver_->Commands().empty());

  driver_->is_torque_failing = false;
  EXPECT_TRUE(interface_->SetMotorPower(false));
}

TEST_F(DriverMotorInterfaceTest, DriverTorqueOffAfterEnableDisconnectsAndTriggersCleanup)
{
  PublishAndSettle();
  ASSERT_TRUE(interface_->SetMotorPower(true));
  std::this_thread::sleep_for(150ms);   // torque_guard_seconds가 지나도록
  PublishAndSettle();
  ASSERT_TRUE(interface_->IsConnected());
  driver_->ClearTorqueRequests();
  driver_->ClearCommands();

  // 드라이버가 스스로 토크를 끔 → 다음 states에서 연결 끊김으로 본다
  driver_->SetTorqueOn(false);
  PublishAndSettle();
  EXPECT_FALSE(interface_->IsConnected());

  // 다음 쓰기에서 한 번만 속도 0 → 토크 끔
  interface_->WriteWheelSpeeds({1.0, 1.0, 1.0});
  EXPECT_FALSE(interface_->IsMotorPowerEnabled());
  EXPECT_EQ(driver_->TorqueRequests(), (std::vector<bool>{false}));
  ASSERT_TRUE(WaitFor([this]() {return !driver_->Commands().empty();}, 1000ms));
  for (const auto & record : driver_->Commands()) {
    EXPECT_TRUE(IsZero(record.message));
  }

  // 고장 표시는 다음 전원 요청까지 남는다
  PublishAndSettle();
  EXPECT_FALSE(interface_->IsConnected());
  ASSERT_TRUE(interface_->SetMotorPower(false));
  PublishAndSettle();
  EXPECT_TRUE(interface_->IsConnected());
}

TEST_F(DriverMotorInterfaceTest, TorqueOffStampedBeforeGuardTimeIsIgnored)
{
  PublishAndSettle();
  const rclcpp::Time before_enable = driver_->now();
  ASSERT_TRUE(interface_->SetMotorPower(true));
  const rclcpp::Time after_enable = driver_->now();

  // 켜기 전에 찍혀 늦게 도착한 메시지
  driver_->PublishStates(before_enable, false);
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(interface_->IsConnected());

  // 켜기 직후 (guard 0.1 s 안)에 찍힌 메시지
  driver_->PublishStates(after_enable, false);
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(interface_->IsConnected());

  // guard 뒤에 찍힌 메시지는 실제 토크 꺼짐이다
  driver_->PublishStates(after_enable + rclcpp::Duration::from_seconds(0.2), false);
  std::this_thread::sleep_for(60ms);
  EXPECT_FALSE(interface_->IsConnected());
}

TEST_F(DriverMotorInterfaceTest, TorqueOffIsNotCheckedWhilePowerIsOff)
{
  // 전원이 꺼져 있으면 torque_enabled=false가 정상이다
  driver_->PublishStates(std::nullopt, false);
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(interface_->IsConnected());

  ASSERT_TRUE(interface_->SetMotorPower(true));
  ASSERT_TRUE(interface_->SetMotorPower(false));
  driver_->PublishStates(
    driver_->now() + rclcpp::Duration::from_seconds(0.3), false);
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(interface_->IsConnected());
}
