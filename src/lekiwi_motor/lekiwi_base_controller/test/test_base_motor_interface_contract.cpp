#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lekiwi_base_controller/base_controller.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace
{

struct RecordingMotorState
{
  bool is_connected{false};
  std::vector<lekiwi_base_controller::WheelCommand> writes;
};

class SafetyOrderingMotorInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  SafetyOrderingMotorInterface()
  : lekiwi_base_controller::BaseMotorInterface(3.0)
  {
  }

  void Connect() override {}
  bool IsConnected() const override { return true; }
  bool SetMotorPower(bool is_enabled) override
  {
    events.push_back(is_enabled ? "power_on" : "power_off");
    SetMotorPowerState(is_enabled);
    return true;
  }

  std::vector<std::string> events;

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand & command) override
  {
    events.push_back(
      command.left == 0.0 && command.back == 0.0 && command.right == 0.0 ? "zero" : "motion");
  }
};

class ThrowingSafetyMotorInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  ThrowingSafetyMotorInterface(bool is_throw_on_zero, bool is_throw_on_power_off)
  : lekiwi_base_controller::BaseMotorInterface(3.0),
    is_throw_on_zero_(is_throw_on_zero),
    is_throw_on_power_off_(is_throw_on_power_off)
  {
  }

  void Connect() override {}
  bool IsConnected() const override { return true; }
  bool SetMotorPower(bool is_enabled) override
  {
    events.push_back(is_enabled ? "power_on" : "power_off");
    if (!is_enabled && is_throw_on_power_off_) {
      throw std::runtime_error("power off failed");
    }
    SetMotorPowerState(is_enabled);
    return true;
  }

  std::vector<std::string> events;

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand & command) override
  {
    const bool is_zero = command.left == 0.0 && command.back == 0.0 && command.right == 0.0;
    events.push_back(is_zero ? "zero" : "motion");
    if (is_zero && is_throw_on_zero_) {
      throw std::runtime_error("zero failed");
    }
  }

  bool is_throw_on_zero_;
  bool is_throw_on_power_off_;
};

class RecordingMotorInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  explicit RecordingMotorInterface(
    double max_wheel_speed,
    std::shared_ptr<RecordingMotorState> state = std::make_shared<RecordingMotorState>())
  : lekiwi_base_controller::BaseMotorInterface(max_wheel_speed),
    state_(std::move(state))
  {
  }

  void Connect() override { state_->is_connected = true; }

  const RecordingMotorState & GetState() const { return *state_; }

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand & command) override
  {
    state_->writes.push_back(command);
  }

  std::shared_ptr<RecordingMotorState> state_;
};

class ThrowingMotorInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  ThrowingMotorInterface()
  : lekiwi_base_controller::BaseMotorInterface(3.0)
  {
  }

  void Connect() override {}

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand &) override
  {
    throw std::runtime_error("write failed");
  }
};

class ShutdownRecordingMotorInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  ShutdownRecordingMotorInterface()
  : lekiwi_base_controller::BaseMotorInterface(3.0)
  {
  }

  void Connect() override {}

  void Shutdown() noexcept override { is_shutdown_called = true; }

  bool is_shutdown_called{false};

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand &) override
  {
    throw std::runtime_error("stop should not be called directly");
  }
};

class FailingMotorPowerInterface final : public lekiwi_base_controller::BaseMotorInterface
{
public:
  FailingMotorPowerInterface()
  : lekiwi_base_controller::BaseMotorInterface(3.0)
  {
  }

  void Connect() override {}
  bool SetMotorPower(bool) override { return false; }

private:
  void WriteClampedWheelSpeeds(const lekiwi_base_controller::WheelCommand &) override {}
};

}  // namespace

TEST(BaseMotorInterfaceContractTest, RejectsNonPositiveMaxWheelSpeed)
{
  EXPECT_THROW(RecordingMotorInterface(0.0), std::invalid_argument);
  EXPECT_THROW(RecordingMotorInterface(-1.0), std::invalid_argument);
}

TEST(BaseMotorInterfaceContractTest, RejectsNonFiniteMaxWheelSpeed)
{
  EXPECT_THROW(
    RecordingMotorInterface(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(
    RecordingMotorInterface(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(BaseMotorInterfaceContractTest, ConnectMustBeExplicit)
{
  RecordingMotorInterface backend(3.0);

  EXPECT_FALSE(backend.GetState().is_connected);
  EXPECT_FALSE(backend.IsConnected());

  backend.Connect();

  EXPECT_TRUE(backend.GetState().is_connected);
  EXPECT_FALSE(backend.IsConnected());
}

TEST(BaseMotorInterfaceContractTest, DefaultFeedbackReadReturnsNoSample)
{
  RecordingMotorInterface backend(3.0);

  EXPECT_FALSE(backend.ReadWheelFeedback().has_value());
}

TEST(BaseMotorInterfaceContractTest, WriteWheelSpeedsClampsEveryWheel)
{
  RecordingMotorInterface backend(3.0);

  backend.WriteWheelSpeeds({9.0, -4.0, 2.0});

  ASSERT_EQ(backend.GetState().writes.size(), 1u);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().left, 3.0);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().back, -3.0);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().right, 2.0);
}

TEST(BaseMotorInterfaceContractTest, StopWritesZeroForEveryWheel)
{
  RecordingMotorInterface backend(3.0);

  backend.WriteWheelSpeeds({1.0, 2.0, 3.0});
  backend.Stop();

  ASSERT_EQ(backend.GetState().writes.size(), 2u);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().left, 0.0);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().back, 0.0);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().right, 0.0);
}

TEST(BaseMotorInterfaceContractTest, MotorPowerTracksStateAndStopsWhenDisabled)
{
  RecordingMotorInterface backend(3.0);

  EXPECT_TRUE(backend.IsMotorPowerEnabled());

  backend.SetMotorPower(false);

  EXPECT_FALSE(backend.IsMotorPowerEnabled());
  ASSERT_EQ(backend.GetState().writes.size(), 1u);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().left, 0.0);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().back, 0.0);
  EXPECT_DOUBLE_EQ(backend.GetState().writes.back().right, 0.0);

  backend.SetMotorPower(true);

  EXPECT_TRUE(backend.IsMotorPowerEnabled());
  EXPECT_EQ(backend.GetState().writes.size(), 1u);
}

TEST(BaseControllerMotorInterfaceTest, MotorPowerServiceHelperReportsSuccess)
{
  RecordingMotorInterface backend(3.0);

  const auto response = lekiwi_base_controller::SetMotorPowerForService(backend, false);

  EXPECT_TRUE(response.success);
  EXPECT_FALSE(response.message.empty());
  EXPECT_FALSE(backend.IsMotorPowerEnabled());
}

TEST(BaseControllerMotorInterfaceTest, MotorPowerServiceHelperReportsFailure)
{
  FailingMotorPowerInterface backend;

  const auto response = lekiwi_base_controller::SetMotorPowerForService(backend, true);

  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("failed"), std::string::npos);
  EXPECT_EQ(response.message.find("enabled"), std::string::npos);
}

TEST(BaseControllerMotorInterfaceTest, ShutdownHelperStopsMotorInterfaceBeforeTeardown)
{
  const auto state = std::make_shared<RecordingMotorState>();
  RecordingMotorInterface backend(3.0, state);

  lekiwi_base_controller::StopMotorInterfaceForShutdown(&backend);

  ASSERT_FALSE(state->writes.empty());
  EXPECT_DOUBLE_EQ(state->writes.back().left, 0.0);
  EXPECT_DOUBLE_EQ(state->writes.back().back, 0.0);
  EXPECT_DOUBLE_EQ(state->writes.back().right, 0.0);
}

TEST(BaseControllerMotorInterfaceTest, ShutdownHelperUsesShutdownApi)
{
  ShutdownRecordingMotorInterface backend;

  lekiwi_base_controller::StopMotorInterfaceForShutdown(&backend);

  EXPECT_TRUE(backend.is_shutdown_called);
}

TEST(BaseControllerMotorInterfaceTest, ShutdownHelperDoesNotThrowWhenStopFails)
{
  ThrowingMotorInterface backend;

  EXPECT_NO_THROW(lekiwi_base_controller::StopMotorInterfaceForShutdown(&backend));
}

TEST(BaseControllerMotorSafetyTest, StaleFeedbackWritesZeroBeforePowerOffAndLatchesFault)
{
  SafetyOrderingMotorInterface backend;
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnFreshEncoderSample();

  EXPECT_TRUE(lekiwi_base_controller::StopMotorOnEncoderTimeout(
      backend,
      latch,
      std::nullopt,
      rclcpp::Time(9, 0),
      rclcpp::Time(10, 0),
      0.3));

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.IsFaultLatched());
  EXPECT_FALSE(backend.IsMotorPowerEnabled());
}

TEST(BaseControllerMotorSafetyTest, ExpiredWaitWritesZeroBeforePowerOffAndLatchesFault)
{
  SafetyOrderingMotorInterface backend;
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));

  EXPECT_TRUE(lekiwi_base_controller::StopMotorOnEncoderTimeout(
      backend,
      latch,
      rclcpp::Time(9, 0),
      std::nullopt,
      rclcpp::Time(10, 0),
      0.3));

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.IsFaultLatched());
  EXPECT_FALSE(backend.IsMotorPowerEnabled());
}

TEST(BaseControllerMotorSafetyTest, FreshFeedbackDoesNotStopOrPowerOff)
{
  SafetyOrderingMotorInterface backend;
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnFreshEncoderSample();

  EXPECT_FALSE(lekiwi_base_controller::StopMotorOnEncoderTimeout(
      backend,
      latch,
      std::nullopt,
      rclcpp::Time(9, 900000000),
      rclcpp::Time(10, 0),
      0.3));

  EXPECT_TRUE(backend.events.empty());
  EXPECT_FALSE(latch.IsFaultLatched());
}

TEST(BaseControllerMotorSafetyTest, ZeroExceptionStillAttemptsPowerOffAndLatchesFault)
{
  ThrowingSafetyMotorInterface backend(true, false);
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));

  EXPECT_NO_THROW({
    EXPECT_TRUE(lekiwi_base_controller::StopMotorOnEncoderTimeout(
        backend,
        latch,
        rclcpp::Time(9, 0),
        std::nullopt,
        rclcpp::Time(10, 0),
        0.3));
  });

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.IsFaultLatched());
}

TEST(BaseControllerMotorSafetyTest, PowerOffExceptionStillLatchesFault)
{
  ThrowingSafetyMotorInterface backend(false, true);
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));

  EXPECT_NO_THROW({
    EXPECT_TRUE(lekiwi_base_controller::StopMotorOnEncoderTimeout(
        backend,
        latch,
        rclcpp::Time(9, 0),
        std::nullopt,
        rclcpp::Time(10, 0),
        0.3));
  });

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.IsFaultLatched());
}

TEST(BaseControllerMotorSafetyTest, ZeroAndPowerOffExceptionsStillLatchFault)
{
  ThrowingSafetyMotorInterface backend(true, true);
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));

  EXPECT_NO_THROW({
    EXPECT_TRUE(lekiwi_base_controller::StopMotorOnEncoderTimeout(
        backend,
        latch,
        rclcpp::Time(9, 0),
        std::nullopt,
        rclcpp::Time(10, 0),
        0.3));
  });

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.IsFaultLatched());
}
