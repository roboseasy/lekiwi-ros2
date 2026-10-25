#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include <lekiwi_msgs/msg/motor_state.hpp>
#include <lekiwi_msgs/msg/motor_state_array.hpp>

#include "lekiwi_base_controller/base_controller.hpp"
#include "lekiwi_base_controller/driver_motor_interface.hpp"
#include "lekiwi_base_controller/wheel_feedback_estimator.hpp"

namespace
{

using lekiwi_base_controller::EncoderFeedbackSource;
using lekiwi_base_controller::WheelFeedbackConfig;
using lekiwi_base_controller::WheelFeedbackEstimator;
using lekiwi_base_controller::WheelStatesFrame;

constexpr double kAnglePerTick = 2.0 * M_PI / 4096.0;
constexpr int64_t kNanosecondsPerSecond = 1000000000;

// 시각을 직접 넘기는 states 프레임 생성기
class FrameClock
{
public:
  WheelStatesFrame Frame(const std::array<int, 3> & ticks)
  {
    WheelStatesFrame frame;
    frame.stamp_nanoseconds = stamp_nanoseconds_;
    frame.is_online = {true, true, true};
    frame.position_ticks = ticks;
    return frame;
  }

  void Advance(const double seconds)
  {
    stamp_nanoseconds_ += static_cast<int64_t>(std::llround(seconds * kNanosecondsPerSecond));
  }

private:
  int64_t stamp_nanoseconds_{5 * kNanosecondsPerSecond};
};

WheelFeedbackConfig PositionConfig()
{
  WheelFeedbackConfig config;
  config.source = EncoderFeedbackSource::Position;
  config.encoder_read_period = 0.1;
  return config;
}

WheelFeedbackConfig SpeedConfig()
{
  WheelFeedbackConfig config;
  config.source = EncoderFeedbackSource::Speed;
  return config;
}

WheelStatesFrame SpeedFrame(const std::array<double, 3> & velocity)
{
  WheelStatesFrame frame;
  frame.stamp_nanoseconds = kNanosecondsPerSecond;
  frame.is_online = {true, true, true};
  frame.velocity = velocity;
  return frame;
}

lekiwi_msgs::msg::MotorState Motor(
  const uint8_t id, const bool is_online, const int ticks, const double velocity)
{
  lekiwi_msgs::msg::MotorState motor;
  motor.id = id;
  motor.online = is_online;
  motor.position_ticks = ticks;
  motor.velocity = velocity;
  return motor;
}

}  // namespace

/*********************************************************
  position 피드백 (이전 Feetech 위치 추정과 같은 계산)
 *********************************************************/

TEST(WheelFeedbackEstimatorTest, FirstPositionFrameSeedsBaselineAndReturnsNoSample)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());

  EXPECT_FALSE(estimator.Update(clock.Frame({1000, 2000, 3000})).has_value());
}

TEST(WheelFeedbackEstimatorTest, SecondPositionFrameReturnsWindowSpeedAndCurrentRawDelta)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 2000, 3000})).has_value());
  clock.Advance(0.1);

  const auto sample = estimator.Update(clock.Frame({1010, 1990, 3010}));

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  // 첫 샘플이 채운 0 구간 두 개(각 0.1 s) + 이번 구간 0.1 s
  EXPECT_NEAR(sample->wheel_speeds.left, 10.0 * kAnglePerTick / 0.3, 1e-12);
  EXPECT_NEAR(sample->wheel_speeds.back, -10.0 * kAnglePerTick / 0.3, 1e-12);
  EXPECT_NEAR(sample->wheel_speeds.right, 10.0 * kAnglePerTick / 0.3, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->left, 10.0 * kAnglePerTick, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->back, -10.0 * kAnglePerTick, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->right, 10.0 * kAnglePerTick, 1e-12);
}

TEST(WheelFeedbackEstimatorTest, StationaryEncoderDitherStaysSlowAndDoesNotAccumulate)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  const std::array<int, 6> positions{1000, 1005, 1000, 994, 999, 1000};
  ASSERT_FALSE(estimator.Update(clock.Frame({positions[0], positions[0], positions[0]})));
  double accumulated_left_delta = 0.0;

  for (std::size_t i = 1; i < positions.size(); ++i) {
    clock.Advance(0.1);
    const auto sample = estimator.Update(clock.Frame({positions[i], positions[i], positions[i]}));
    ASSERT_TRUE(sample.has_value());
    ASSERT_TRUE(sample->wheel_position_delta.has_value());
    EXPECT_LE(std::abs(sample->wheel_speeds.left), 0.05);
    EXPECT_LE(std::abs(sample->wheel_speeds.back), 0.05);
    EXPECT_LE(std::abs(sample->wheel_speeds.right), 0.05);
    accumulated_left_delta += sample->wheel_position_delta->left;
  }
  EXPECT_NEAR(accumulated_left_delta, 0.0, 1e-12);
}

TEST(WheelFeedbackEstimatorTest, MonotonicSingleTicksAreAllPreservedInPositionDeltas)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));
  double total_delta = 0.0;

  for (int position = 1001; position <= 1010; ++position) {
    clock.Advance(0.1);
    const auto sample = estimator.Update(clock.Frame({position, position, position}));
    ASSERT_TRUE(sample.has_value());
    ASSERT_TRUE(sample->wheel_position_delta.has_value());
    EXPECT_NEAR(sample->wheel_position_delta->left, kAnglePerTick, 1e-12);
    total_delta += sample->wheel_position_delta->left;
  }
  EXPECT_NEAR(total_delta, 10.0 * kAnglePerTick, 1e-12);
}

TEST(WheelFeedbackEstimatorTest, ThreeIntervalWindowUsesActualStampElapsedTime)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  constexpr int kIntervals = 546;
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));
  double total_delta = 0.0;
  std::array<double, 3> last_intervals{};
  std::optional<lekiwi_base_controller::WheelFeedbackSample> last_sample;

  for (int i = 0; i < kIntervals; ++i) {
    const double elapsed = i % 2 == 0 ? 0.10 : 0.12;
    last_intervals[static_cast<std::size_t>(i) % last_intervals.size()] = elapsed;
    clock.Advance(elapsed);
    const int position = 1001 + i;
    last_sample = estimator.Update(clock.Frame({position, position, position}));
    ASSERT_TRUE(last_sample.has_value());
    ASSERT_TRUE(last_sample->wheel_position_delta.has_value());
    total_delta += last_sample->wheel_position_delta->left;
  }

  const double window_time = last_intervals[0] + last_intervals[1] + last_intervals[2];
  EXPECT_NEAR(total_delta, kIntervals * kAnglePerTick, 1e-10);
  EXPECT_NEAR(last_sample->wheel_speeds.left, 3.0 * kAnglePerTick / window_time, 1e-9);
}

TEST(WheelFeedbackEstimatorTest, ThreeFixedReadsAfterMotionMakeWindowVelocityExactlyZero)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));
  std::optional<lekiwi_base_controller::WheelFeedbackSample> sample;
  for (const int position : {1010, 1010, 1010, 1010}) {
    clock.Advance(0.1);
    sample = estimator.Update(clock.Frame({position, position, position}));
    ASSERT_TRUE(sample.has_value());
  }
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.left, 0.0);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.back, 0.0);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.right, 0.0);
}

TEST(WheelFeedbackEstimatorTest, PositionDeltaWrapsAcross4095To0InBothDirections)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({4090, 5, 1000})));
  clock.Advance(0.1);

  const auto sample = estimator.Update(clock.Frame({5, 4090, 1000}));

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  EXPECT_NEAR(sample->wheel_position_delta->left, 11.0 * kAnglePerTick, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->back, -11.0 * kAnglePerTick, 1e-12);
  EXPECT_DOUBLE_EQ(sample->wheel_position_delta->right, 0.0);
  EXPECT_NEAR(sample->wheel_speeds.left, 11.0 * kAnglePerTick / 0.3, 1e-12);
}

TEST(WheelFeedbackEstimatorTest, WrapPositionDeltaKeepsHalfRevolutionBoundary)
{
  EXPECT_EQ(lekiwi_base_controller::WrapPositionDelta(0, 4095, 4096), 1);
  EXPECT_EQ(lekiwi_base_controller::WrapPositionDelta(4095, 0, 4096), -1);
  EXPECT_EQ(lekiwi_base_controller::WrapPositionDelta(2048, 0, 4096), 2048);
  EXPECT_EQ(lekiwi_base_controller::WrapPositionDelta(2049, 0, 4096), -2047);
  EXPECT_EQ(lekiwi_base_controller::WrapPositionDelta(0, 2049, 4096), 2047);
}

TEST(WheelFeedbackEstimatorTest, WheelDirectionAndOdomScaleApplyToSpeedAndDelta)
{
  FrameClock clock;
  auto config = PositionConfig();
  config.wheel_directions = {2.0, -2.0, 0.5};
  config.encoder_odom_scale = 0.5;
  WheelFeedbackEstimator estimator(config);
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));
  clock.Advance(0.1);

  const auto sample = estimator.Update(clock.Frame({1010, 1010, 1010}));

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  const std::array<double, 3> speeds{
    sample->wheel_speeds.left, sample->wheel_speeds.back, sample->wheel_speeds.right};
  const std::array<double, 3> deltas{
    sample->wheel_position_delta->left,
    sample->wheel_position_delta->back,
    sample->wheel_position_delta->right};
  for (std::size_t i = 0; i < 3; ++i) {
    // 계수 = encoder_odom_scale / wheel_directions[i]
    const double expected_delta = 10.0 * kAnglePerTick / config.wheel_directions[i] * 0.5;
    EXPECT_NEAR(deltas[i], expected_delta, 1e-12);
    EXPECT_NEAR(speeds[i], expected_delta / 0.3, 1e-12);
  }
  EXPECT_LT(deltas[1], 0.0);
}

TEST(WheelFeedbackEstimatorTest, OfflineWheelReturnsNoSampleAndPreservesBaselineAndWindow)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));

  clock.Advance(0.1);
  auto offline = clock.Frame({1010, 1010, 1010});
  offline.is_online[1] = false;
  EXPECT_FALSE(estimator.Update(offline).has_value());

  clock.Advance(0.1);
  const auto sample = estimator.Update(clock.Frame({1020, 1020, 1020}));

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  // 실패한 읽기는 기준점을 바꾸지 않으므로 이번 구간은 0.2 s, 20 tick
  EXPECT_NEAR(sample->wheel_position_delta->left, 20.0 * kAnglePerTick, 1e-12);
  EXPECT_NEAR(sample->wheel_speeds.left, 20.0 * kAnglePerTick / 0.4, 1e-12);
}

TEST(WheelFeedbackEstimatorTest, NonIncreasingStampOrOutOfRangeTicksGiveNoSample)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));

  // 같은 시각 (elapsed = 0)
  EXPECT_FALSE(estimator.Update(clock.Frame({1010, 1010, 1010})).has_value());
  clock.Advance(0.1);
  EXPECT_FALSE(estimator.Update(clock.Frame({1010, 4096, 1010})).has_value());
  EXPECT_FALSE(estimator.Update(clock.Frame({1010, -1, 1010})).has_value());

  const auto sample = estimator.Update(clock.Frame({1010, 1010, 1010}));
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->wheel_position_delta->left, 10.0 * kAnglePerTick, 1e-12);
}

TEST(WheelFeedbackEstimatorTest, ResetMakesNextFrameABaselineAgain)
{
  FrameClock clock;
  WheelFeedbackEstimator estimator(PositionConfig());
  ASSERT_FALSE(estimator.Update(clock.Frame({1000, 1000, 1000})));
  clock.Advance(0.1);
  ASSERT_TRUE(estimator.Update(clock.Frame({1010, 1010, 1010})).has_value());

  estimator.Reset();
  clock.Advance(0.1);
  EXPECT_FALSE(estimator.Update(clock.Frame({1020, 1020, 1020})).has_value());
  clock.Advance(0.1);
  const auto sample = estimator.Update(clock.Frame({1030, 1030, 1030}));
  ASSERT_TRUE(sample.has_value());
  // 창도 새로 시작한다 (0 구간 두 개 + 이번 구간)
  EXPECT_NEAR(sample->wheel_speeds.left, 10.0 * kAnglePerTick / 0.3, 1e-12);
}

/*********************************************************
  speed 피드백
 *********************************************************/

TEST(WheelFeedbackEstimatorTest, SpeedFeedbackAppliesDeadbandEquivalentToFiftyTicks)
{
  WheelFeedbackEstimator estimator(SpeedConfig());

  // 드라이버 velocity = tick × 2pi / 4096
  const auto sample = estimator.Update(
    SpeedFrame({50.0 * kAnglePerTick, -50.0 * kAnglePerTick, 51.0 * kAnglePerTick}));

  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->wheel_position_delta.has_value());
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.left, 0.0);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.back, 0.0);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.right, 51.0 * kAnglePerTick);
}

TEST(WheelFeedbackEstimatorTest, SpeedFeedbackDividesByDirectionAndAppliesScale)
{
  auto config = SpeedConfig();
  config.wheel_directions = {-1.0, 2.0, 1.0};
  config.encoder_odom_scale = 0.5;
  config.encoder_speed_deadband = 0.0;
  WheelFeedbackEstimator estimator(config);

  const auto sample = estimator.Update(SpeedFrame({1.0, -1.0, 0.4}));

  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.left, -0.5);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.back, -0.25);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.right, 0.2);
}

TEST(WheelFeedbackEstimatorTest, SpeedFeedbackOfflineWheelGivesNoSample)
{
  WheelFeedbackEstimator estimator(SpeedConfig());
  auto frame = SpeedFrame({1.0, 1.0, 1.0});
  frame.is_online[2] = false;

  EXPECT_FALSE(estimator.Update(frame).has_value());
}

/*********************************************************
  드라이버 메시지 변환
 *********************************************************/

TEST(DriverMessageMappingTest, ExtractsWheelsInModelOrderAndTreatsMissingIdAsOffline)
{
  lekiwi_msgs::msg::MotorStateArray states;
  states.header.stamp = rclcpp::Time(3, 250000000);
  states.motors = {
    Motor(1, true, 111, 0.1),   // 팔은 무시한다
    Motor(7, true, 700, 0.7),
    Motor(8, true, 800, 0.8),
    Motor(9, false, 0, 0.0)};

  const auto frame = lekiwi_base_controller::ExtractWheelStatesFrame(states, {8, 9, 7});

  EXPECT_EQ(frame.stamp_nanoseconds, 3250000000);
  EXPECT_EQ(frame.is_online, (std::array<bool, 3>{true, false, true}));
  EXPECT_EQ(frame.position_ticks, (std::array<int, 3>{800, 0, 700}));
  EXPECT_EQ(frame.velocity, (std::array<double, 3>{0.8, 0.0, 0.7}));

  states.motors.pop_back();   // ID 9 없음
  const auto missing = lekiwi_base_controller::ExtractWheelStatesFrame(states, {8, 9, 7});
  EXPECT_FALSE(missing.is_online[1]);
}

TEST(DriverMessageMappingTest, WheelCommandUsesModelOrderIdsAndDirectionSigns)
{
  const auto msg = lekiwi_base_controller::MakeWheelCommandMessage(
    lekiwi_base_controller::WheelCommand{1.0, -0.5, 0.25},
    {8, 9, 7},
    {1.0, -1.0, -2.0},
    rclcpp::Time(4, 0));

  EXPECT_EQ(rclcpp::Time(msg.header.stamp).nanoseconds(), rclcpp::Time(4, 0).nanoseconds());
  EXPECT_EQ(msg.ids, (std::vector<uint8_t>{8, 9, 7}));
  ASSERT_EQ(msg.velocity.size(), 3u);
  EXPECT_DOUBLE_EQ(msg.velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(msg.velocity[1], 0.5);
  EXPECT_DOUBLE_EQ(msg.velocity[2], -0.5);
}

TEST(DriverMessageMappingTest, CommandAndFeedbackDirectionsAreInverse)
{
  // 명령은 × 방향, 피드백은 ÷ 방향이므로 왕복하면 모델 속도로 돌아온다
  const std::array<double, 3> directions{-1.0, 1.0, -1.0};
  const lekiwi_base_controller::WheelCommand command{0.3, -0.2, 0.1};
  const auto motor = lekiwi_base_controller::ToMotorVelocities(command, directions);
  auto config = SpeedConfig();
  config.wheel_directions = directions;
  config.encoder_speed_deadband = 0.0;
  WheelFeedbackEstimator estimator(config);

  const auto sample = estimator.Update(SpeedFrame(motor));

  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.left, command.left);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.back, command.back);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.right, command.right);
}

TEST(DriverMessageMappingTest, DriverConfigCopiesValidatedParameters)
{
  lekiwi_base_controller::BaseControllerParameters params;
  params.motor_backend = "driver";
  params.is_motor_write_enabled = true;
  params.wheel_directions = {1.0, -1.0, 1.0};
  params.encoder_feedback_source = "speed";
  params.encoder_odom_scale = 0.9;
  params.states_topic = "custom/states";

  const auto config = lekiwi_base_controller::MakeDriverMotorConfig(params);

  EXPECT_EQ(config.wheel_ids, (std::array<uint8_t, 3>{8, 9, 7}));
  EXPECT_EQ(config.feedback.wheel_directions, (std::array<double, 3>{1.0, -1.0, 1.0}));
  EXPECT_EQ(config.feedback.source, EncoderFeedbackSource::Speed);
  EXPECT_EQ(config.feedback.position_ticks_per_revolution, 4096);
  EXPECT_DOUBLE_EQ(config.feedback.encoder_odom_scale, 0.9);
  EXPECT_DOUBLE_EQ(config.feedback.encoder_read_period, 0.1);
  EXPECT_EQ(config.states_topic, "custom/states");
  EXPECT_EQ(config.wheel_command_topic, "lekiwi/motor/wheel_command");
  EXPECT_EQ(config.wheel_torque_service, "lekiwi/motor/wheel_torque");
  EXPECT_DOUBLE_EQ(config.driver_states_timeout, 0.5);
  EXPECT_DOUBLE_EQ(config.torque_service_timeout, 1.0);
}
