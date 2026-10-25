#include <gtest/gtest.h>

#include <deque>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lekiwi_base_controller/base_controller.hpp"
#include "lekiwi_base_controller/motor_safety_coordinator.hpp"

namespace
{

class FakeMotorInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  FakeMotorInterface()
  : lekiwi_base_controller::BaseMotorInterface(5.0)
  {
  }

  void Connect() override
  {
    events.push_back("connect");
    is_connected = is_connect_ok;
  }

  bool IsConnected() const override { return is_connected; }

  std::optional<lekiwi_base_controller::WheelFeedbackSample> ReadWheelFeedback() override
  {
    events.push_back("read");
    if (on_read) {
      on_read();
    }
    if (is_read_throwing) {
      throw std::runtime_error("read failed");
    }
    if (reads.empty()) {
      return std::nullopt;
    }
    auto result = reads.front();
    reads.pop_front();
    return result;
  }

  bool SetMotorPower(bool is_enabled) override
  {
    events.push_back(is_enabled ? "power_on" : "power_off");
    if (is_enabled && !is_power_enable_ok) {
      return false;
    }
    if (!is_enabled && !is_power_disable_ok) {
      return false;
    }
    SetMotorPowerState(is_enabled);
    return true;
  }

  bool is_connected{true};
  bool is_connect_ok{true};
  bool is_power_enable_ok{true};
  bool is_power_disable_ok{true};
  bool is_disconnect_on_write{false};
  bool is_read_throwing{false};
  std::deque<std::optional<lekiwi_base_controller::WheelFeedbackSample>> reads;
  std::vector<std::string> events;
  std::function<void()> on_read;

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand & command) override
  {
    const bool is_zero = command.left == 0.0 && command.back == 0.0 && command.right == 0.0;
    events.push_back(is_zero ? "zero" : "motion");
    if (is_disconnect_on_write) {
      is_connected = false;
      SetMotorPowerState(false);
    }
  }
};

lekiwi_base_controller::WheelFeedbackSample Feedback(
  const lekiwi_base_controller::WheelCommand & wheel_speeds,
  std::optional<lekiwi_base_controller::WheelPositionDelta> wheel_position_delta = std::nullopt)
{
  return lekiwi_base_controller::WheelFeedbackSample{wheel_speeds, wheel_position_delta};
}

}  // namespace

TEST(MotorSafetyCoordinatorTest, FreshReadUnlocksMotionAndFailedReadRetainsReadinessUntilTimeout)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });

  backend.reads.push_back(std::nullopt);
  now = rclcpp::Time(0, 100000000);
  auto update = coordinator.BeforeCommand(now);
  coordinator.SendWheelCommand({1.0, 0.0, 0.0});
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read", "zero"}));

  backend.events.clear();
  backend.reads.push_back(
    Feedback({0.1, 0.2, 0.3}, lekiwi_base_controller::WheelPositionDelta{0.01, 0.02, 0.03}));
  now = rclcpp::Time(0, 200000000);
  update = coordinator.BeforeCommand(now);
  coordinator.SendWheelCommand({1.0, 0.0, 0.0});
  EXPECT_TRUE(update.is_readiness_changed);
  EXPECT_TRUE(coordinator.IsMotorReady());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read", "motion"}));

  backend.events.clear();
  backend.reads.push_back(std::nullopt);
  now = rclcpp::Time(0, 300000000);
  update = coordinator.BeforeCommand(now);
  coordinator.SendWheelCommand({1.0, 0.0, 0.0});
  EXPECT_FALSE(update.is_readiness_changed);
  EXPECT_TRUE(coordinator.IsMotorReady());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read", "motion"}));
}

TEST(MotorSafetyCoordinatorTest, AcceptedDeltaIsReturnedOnlyOnItsReadCycle)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  const lekiwi_base_controller::WheelPositionDelta expected_delta{0.01, -0.02, 0.03};
  backend.reads.push_back(Feedback({0.4, -0.5, 0.6}, expected_delta));

  now = rclcpp::Time(0, 100000000);
  const auto read_update = coordinator.BeforeCommand(now);
  ASSERT_TRUE(read_update.wheel_position_delta.has_value());
  EXPECT_DOUBLE_EQ(read_update.wheel_position_delta->left, expected_delta.left);
  EXPECT_DOUBLE_EQ(read_update.wheel_position_delta->back, expected_delta.back);
  EXPECT_DOUBLE_EQ(read_update.wheel_position_delta->right, expected_delta.right);

  for (const int64_t hold_ns : {120000000, 140000000, 160000000, 180000000}) {
    now = rclcpp::Time(0, hold_ns);
    const auto hold_update = coordinator.BeforeCommand(now);
    EXPECT_FALSE(hold_update.wheel_position_delta.has_value());
    const auto fresh_speeds = coordinator.GetFreshEncoderWheelSpeeds(now);
    ASSERT_TRUE(fresh_speeds.has_value());
    EXPECT_DOUBLE_EQ(fresh_speeds->left, 0.4);
    EXPECT_DOUBLE_EQ(fresh_speeds->back, -0.5);
    EXPECT_DOUBLE_EQ(fresh_speeds->right, 0.6);
  }
}

TEST(MotorSafetyCoordinatorTest, AlternatingReadsDeliverEveryDeltaOnceForSixtySeconds)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  constexpr int64_t kControlPeriodNs = 20000000;
  constexpr int64_t kMinimumDurationNs = 60000000000;
  int64_t due_time_ns = 0;
  std::size_t sample_index = 0;
  lekiwi_base_controller::WheelPositionDelta expected_sum{};
  lekiwi_base_controller::WheelPositionDelta delivered_sum{};

  while (due_time_ns < kMinimumDurationNs) {
    const int64_t read_period_ns = sample_index % 2 == 0 ? 100000000 : 120000000;
    const int64_t previous_due_time_ns = due_time_ns;
    due_time_ns += read_period_ns;
    const double raw_ticks = static_cast<double>(static_cast<int>(sample_index % 9) - 4);
    const double scale = 0.0025;
    const lekiwi_base_controller::WheelPositionDelta expected_delta{
      raw_ticks * scale,
      -2.0 * raw_ticks * scale,
      0.5 * raw_ticks * scale};
    backend.reads.push_back(Feedback({0.4, -0.5, 0.6}, expected_delta));
    expected_sum.left += expected_delta.left;
    expected_sum.back += expected_delta.back;
    expected_sum.right += expected_delta.right;

    for (int64_t hold_time_ns = previous_due_time_ns + kControlPeriodNs;
      hold_time_ns < due_time_ns &&
      hold_time_ns - previous_due_time_ns < 100000000;
      hold_time_ns += kControlPeriodNs)
    {
      now = rclcpp::Time(hold_time_ns);
      const auto hold_update = coordinator.BeforeCommand(now);
      EXPECT_FALSE(hold_update.wheel_position_delta.has_value()) << hold_time_ns;
    }

    now = rclcpp::Time(due_time_ns);
    const auto read_update = coordinator.BeforeCommand(now);
    ASSERT_TRUE(read_update.wheel_position_delta.has_value()) << due_time_ns;
    delivered_sum.left += read_update.wheel_position_delta->left;
    delivered_sum.back += read_update.wheel_position_delta->back;
    delivered_sum.right += read_update.wheel_position_delta->right;
    ++sample_index;
  }

  EXPECT_GE(due_time_ns, kMinimumDurationNs);
  EXPECT_GT(sample_index, 500u);
  EXPECT_DOUBLE_EQ(delivered_sum.left, expected_sum.left);
  EXPECT_DOUBLE_EQ(delivered_sum.back, expected_sum.back);
  EXPECT_DOUBLE_EQ(delivered_sum.right, expected_sum.right);
  EXPECT_TRUE(backend.reads.empty());
}

TEST(MotorSafetyCoordinatorTest, AlreadyPoweredConstructorTimesOutWaitingFromInitialTime)
{
  FakeMotorInterface backend;
  rclcpp::Time now(5, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 1.0, 0.3, now, [&now]() { return now; });

  now = rclcpp::Time(5, 300000001);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, SuccessfulPowerEnableTimesOutWaitingFromRequestTime)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  ASSERT_TRUE(coordinator.SetMotorPower(false, now).success);
  const rclcpp::Time request_time(1, 0);
  ASSERT_TRUE(coordinator.SetMotorPower(true, request_time).success);

  backend.events.clear();
  now = rclcpp::Time(1, 100000000);
  EXPECT_FALSE(coordinator.BeforeCommand(now).is_encoder_timeout);
  now = rclcpp::Time(1, 200000000);
  EXPECT_FALSE(coordinator.BeforeCommand(now).is_encoder_timeout);
  now = rclcpp::Time(1, 300000001);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_EQ(
    backend.events,
    (std::vector<std::string>{"read", "read", "zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, TimeoutGateRunsBeforeAReadThatIsNotYetDue)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });

  now = rclcpp::Time(0, 290000000);
  EXPECT_FALSE(coordinator.BeforeCommand(now).is_encoder_timeout);
  ASSERT_EQ(backend.events, (std::vector<std::string>{"read"}));

  backend.events.clear();
  now = rclcpp::Time(0, 310000000);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_EQ(update.encoder_timeout_stage, lekiwi_base_controller::EncoderTimeoutStage::BeforeRead);
  EXPECT_EQ(update.encoder_read_status, lekiwi_base_controller::EncoderReadStatus::NotAttempted);
  EXPECT_NEAR(update.encoder_sample_age_seconds, 0.31, 1e-9);
  EXPECT_EQ(update.consecutive_failed_reads, 1u);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, ReadCompletingAfterWaitDeadlineIsDiscarded)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.reads.push_back(
    Feedback({0.1, 0.2, 0.3}, lekiwi_base_controller::WheelPositionDelta{0.01, 0.02, 0.03}));
  backend.on_read = [&now]() { now = rclcpp::Time(0, 300000001); };

  now = rclcpp::Time(0, 299000000);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_EQ(update.encoder_timeout_stage, lekiwi_base_controller::EncoderTimeoutStage::AfterRead);
  EXPECT_EQ(update.encoder_read_status, lekiwi_base_controller::EncoderReadStatus::Sample);
  EXPECT_NEAR(update.encoder_sample_age_seconds, 0.300000001, 1e-9);
  EXPECT_NEAR(update.encoder_read_duration_seconds, 0.001000001, 1e-9);
  EXPECT_EQ(update.consecutive_failed_reads, 0u);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_FALSE(coordinator.GetFreshEncoderWheelSpeeds(now).has_value());
  EXPECT_FALSE(update.wheel_position_delta.has_value());
  EXPECT_EQ(
    backend.events,
    (std::vector<std::string>{"read", "zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, ThrowingReadCrossingWaitDeadlineFailsClosed)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.is_read_throwing = true;
  backend.on_read = [&now]() { now = rclcpp::Time(0, 300000001); };

  now = rclcpp::Time(0, 299000000);
  lekiwi_base_controller::MotorSafetyUpdate update;
  EXPECT_NO_THROW(update = coordinator.BeforeCommand(now));

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_EQ(update.encoder_timeout_stage, lekiwi_base_controller::EncoderTimeoutStage::AfterRead);
  EXPECT_EQ(update.encoder_read_status, lekiwi_base_controller::EncoderReadStatus::Exception);
  EXPECT_EQ(update.consecutive_failed_reads, 1u);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(
    backend.events,
    (std::vector<std::string>{"read", "zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, ThrowingReadBeforeDeadlineRemainsFailedSample)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.is_read_throwing = true;

  now = rclcpp::Time(0, 100000000);
  lekiwi_base_controller::MotorSafetyUpdate update;
  EXPECT_NO_THROW(update = coordinator.BeforeCommand(now));

  EXPECT_FALSE(update.is_encoder_timeout);
  EXPECT_FALSE(update.is_readiness_changed);
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_FALSE(coordinator.IsFaultLatched());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read"}));
}

TEST(MotorSafetyCoordinatorTest, StaleReadySampleTimesOutBeforeQueuedSuccessCanHealIt)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.reads.push_back(Feedback({0.1, 0.2, 0.3}));
  now = rclcpp::Time(0, 100000000);
  ASSERT_TRUE(coordinator.BeforeCommand(now).is_readiness_changed);
  ASSERT_TRUE(coordinator.IsMotorReady());

  backend.events.clear();
  backend.reads.push_back(
    Feedback({0.4, 0.5, 0.6}, lekiwi_base_controller::WheelPositionDelta{0.04, 0.05, 0.06}));
  now = rclcpp::Time(0, 400000001);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_FALSE(update.wheel_position_delta.has_value());
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, DefaultDeadlineToleratesBriefDelayButStopsOnMissingFeedback)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  const lekiwi_base_controller::BaseControllerParameters defaults;
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, defaults.encoder_read_period,
    defaults.encoder_sample_timeout, now, [&now]() { return now; });
  backend.reads.push_back(Feedback({}));
  now = rclcpp::Time(0, 100000000);
  ASSERT_TRUE(coordinator.BeforeCommand(now).is_readiness_changed);

  backend.reads.push_back(Feedback({}));
  now = rclcpp::Time(0, 424000000);
  const auto delayed_update = coordinator.BeforeCommand(now);
  EXPECT_FALSE(delayed_update.is_encoder_timeout);
  EXPECT_EQ(delayed_update.encoder_read_status, lekiwi_base_controller::EncoderReadStatus::Sample);
  EXPECT_TRUE(coordinator.IsMotorReady());

  backend.events.clear();
  now = rclcpp::Time(0, 924000001);
  const auto timeout_update = coordinator.BeforeCommand(now);
  EXPECT_TRUE(timeout_update.is_encoder_timeout);
  EXPECT_EQ(timeout_update.encoder_timeout_stage, lekiwi_base_controller::EncoderTimeoutStage::BeforeRead);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, PostReadStaleFeedbackStopsBeforePowerOffAndBlocksMotion)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.reads.push_back(Feedback({}));
  now = rclcpp::Time(0, 100000000);
  coordinator.BeforeCommand(now);
  ASSERT_TRUE(coordinator.IsMotorReady());

  backend.events.clear();
  backend.reads.push_back(std::nullopt);
  backend.on_read = [&now]() { now = rclcpp::Time(0, 450000000); };
  now = rclcpp::Time(0, 200000000);
  const auto update = coordinator.BeforeCommand(rclcpp::Time(0, 200000000));
  coordinator.SendWheelCommand({1.0, 0.0, 0.0});

  EXPECT_TRUE(update.is_encoder_timeout);
  EXPECT_EQ(update.encoder_timeout_stage, lekiwi_base_controller::EncoderTimeoutStage::AfterRead);
  EXPECT_EQ(update.encoder_read_status, lekiwi_base_controller::EncoderReadStatus::NoSample);
  EXPECT_NEAR(update.encoder_read_duration_seconds, 0.25, 1e-9);
  EXPECT_EQ(update.consecutive_failed_reads, 1u);
  EXPECT_TRUE(update.is_readiness_changed);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read", "zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, SuccessfulDisableClearsEncoderDeadlineState)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.reads.push_back(Feedback({0.1, 0.2, 0.3}));
  now = rclcpp::Time(0, 100000000);
  ASSERT_TRUE(coordinator.BeforeCommand(now).is_readiness_changed);

  now = rclcpp::Time(0, 200000000);
  ASSERT_TRUE(coordinator.SetMotorPower(false, now).success);
  now = rclcpp::Time(0, 600000000);
  EXPECT_FALSE(coordinator.BeforeCommand(now).is_encoder_timeout);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read", "power_off"}));

  ASSERT_TRUE(coordinator.SetMotorPower(true, now).success);
  backend.events.clear();
  now = rclcpp::Time(0, 900000000);
  EXPECT_FALSE(coordinator.BeforeCommand(now).is_encoder_timeout);
  now = rclcpp::Time(0, 900000001);
  EXPECT_TRUE(coordinator.BeforeCommand(now).is_encoder_timeout);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read", "zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, DisarmedStaleFeedbackDoesNotLatchFault)
{
  FakeMotorInterface backend;
  ASSERT_TRUE(backend.SetMotorPower(false));
  backend.events.clear();
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });

  backend.reads.push_back(Feedback({0.1, 0.2, 0.3}));
  now = rclcpp::Time(0, 100000000);
  EXPECT_FALSE(coordinator.BeforeCommand(now).is_encoder_timeout);
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_FALSE(coordinator.IsFaultLatched());

  backend.events.clear();
  now = rclcpp::Time(0, 500000000);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_FALSE(update.is_encoder_timeout);
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_FALSE(coordinator.IsFaultLatched());
  EXPECT_EQ(backend.events, (std::vector<std::string>{"read"}));
}

TEST(MotorSafetyCoordinatorTest, FaultRecoveryRequiresDisableAndFreshSample)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.reads.push_back(Feedback({}));
  now = rclcpp::Time(0, 100000000);
  coordinator.BeforeCommand(now);
  backend.reads.push_back(std::nullopt);
  backend.on_read = [&now]() { now = rclcpp::Time(0, 500000000); };
  coordinator.BeforeCommand(rclcpp::Time(0, 200000000));
  ASSERT_TRUE(coordinator.IsFaultLatched());

  auto response = coordinator.SetMotorPower(true, now);
  EXPECT_FALSE(response.success);

  response = coordinator.SetMotorPower(false, now);
  EXPECT_TRUE(response.success);
  response = coordinator.SetMotorPower(true, now);
  EXPECT_TRUE(response.success);
  EXPECT_FALSE(coordinator.IsMotorReady());

  backend.on_read = {};
  backend.reads.push_back(Feedback({}));
  now = rclcpp::Time(0, 600000000);
  const auto update = coordinator.BeforeCommand(now);
  EXPECT_TRUE(update.is_readiness_changed);
  EXPECT_TRUE(coordinator.IsMotorReady());
}

TEST(MotorSafetyCoordinatorTest, BackendEnableFailureStopsDisarmsAndLatchesFault)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.is_power_enable_ok = false;

  const auto response = coordinator.SetMotorPower(true, now);

  EXPECT_FALSE(response.success);
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(
    backend.events,
    (std::vector<std::string>{"power_on", "zero", "power_off"}));
}

TEST(MotorSafetyCoordinatorTest, WaitingStateCommandWriteFailureLatchesFault)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.is_disconnect_on_write = true;

  coordinator.SendWheelCommand({1.0, 0.0, 0.0});

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero"}));
  EXPECT_TRUE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
}

TEST(MotorSafetyCoordinatorTest, FailedDisableDoesNotAuthorizeFaultRecovery)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.is_power_disable_ok = false;

  auto response = coordinator.SetMotorPower(false, now);
  EXPECT_FALSE(response.success);
  EXPECT_TRUE(coordinator.IsFaultLatched());

  response = coordinator.SetMotorPower(true, now);
  EXPECT_FALSE(response.success);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"power_off"}));

  backend.is_power_disable_ok = true;
  response = coordinator.SetMotorPower(false, now);
  EXPECT_TRUE(response.success);
  response = coordinator.SetMotorPower(true, now);
  EXPECT_TRUE(response.success);
  EXPECT_FALSE(coordinator.IsFaultLatched());
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_EQ(
    backend.events,
    (std::vector<std::string>{"power_off", "power_off", "power_on"}));
}

TEST(MotorSafetyCoordinatorTest, DisconnectAfterReadyRequestsFalseReadinessPublication)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, true, true, 0.1, 0.3, now, [&now]() { return now; });
  backend.reads.push_back(Feedback({}));
  now = rclcpp::Time(0, 100000000);
  ASSERT_TRUE(coordinator.BeforeCommand(now).is_readiness_changed);
  ASSERT_TRUE(coordinator.IsMotorReady());

  backend.is_connected = false;
  now = rclcpp::Time(0, 150000000);
  const auto update = coordinator.BeforeCommand(now);

  EXPECT_TRUE(update.is_readiness_changed);
  EXPECT_FALSE(coordinator.IsMotorReady());
  EXPECT_TRUE(coordinator.IsFaultLatched());
}

TEST(MotorSafetyCoordinatorTest, MockServiceBehaviorRemainsAvailableButNeverReady)
{
  FakeMotorInterface backend;
  rclcpp::Time now(0, 0);
  lekiwi_base_controller::MotorSafetyCoordinator coordinator(
    backend, false, false, 0.1, 0.3, now, [&now]() { return now; });

  EXPECT_TRUE(coordinator.SetMotorPower(false, now).success);
  EXPECT_TRUE(coordinator.SetMotorPower(true, now).success);
  EXPECT_FALSE(coordinator.IsMotorReady());
}

TEST(MotorSafetyCoordinatorTest, MotorReadyQosIsReliableTransientLocalDepthOne)
{
  const auto profile = lekiwi_base_controller::MakeMotorReadyQos().get_rmw_qos_profile();

  EXPECT_EQ(profile.depth, 1u);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}
