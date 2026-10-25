#include <gtest/gtest.h>

#include "lekiwi_base_controller/base_controller.hpp"
#include "lekiwi_base_controller/motor_readiness.hpp"

TEST(MotorReadinessLatchTest, DisarmedPermitsOnlyZeroCommand)
{
  lekiwi_base_controller::MotorReadinessLatch latch;

  EXPECT_FALSE(latch.Permits(lekiwi_base_controller::WheelCommand{1.0, 0.0, 0.0}));
  EXPECT_TRUE(latch.Permits(lekiwi_base_controller::WheelCommand{}));
}

TEST(MotorReadinessLatchTest, FreshEncoderSampleMakesConnectedPoweredHardwareReady)
{
  lekiwi_base_controller::MotorReadinessLatch latch;

  EXPECT_TRUE(latch.OnPowerEnabled(true));
  EXPECT_FALSE(latch.IsReady(true, true, true));
  latch.OnFreshEncoderSample();

  EXPECT_TRUE(latch.IsReady(true, true, true));
  EXPECT_TRUE(latch.Permits(lekiwi_base_controller::WheelCommand{1.0, 0.0, 0.0}));
}

TEST(MotorReadinessLatchTest, EncoderTimeoutLatchesFaultAndBlocksMotion)
{
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnFreshEncoderSample();

  latch.OnEncoderTimeout();

  EXPECT_TRUE(latch.IsFaultLatched());
  EXPECT_FALSE(latch.Permits(lekiwi_base_controller::WheelCommand{1.0, 0.0, 0.0}));
  EXPECT_TRUE(latch.Permits(lekiwi_base_controller::WheelCommand{}));
}

TEST(MotorReadinessLatchTest, PowerCycleThroughDisarmedClearsFaultButRequiresFreshSample)
{
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnFreshEncoderSample();
  latch.OnEncoderTimeout();
  ASSERT_TRUE(latch.IsFaultLatched());

  latch.OnPowerDisabled();
  EXPECT_TRUE(latch.OnPowerEnabled(true));
  EXPECT_FALSE(latch.IsReady(true, true, true));
  latch.OnFreshEncoderSample();

  EXPECT_TRUE(latch.IsReady(true, true, true));
}

TEST(MotorReadinessLatchTest, FaultCannotBeRearmedWithoutPowerDisable)
{
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnBackendFault();

  EXPECT_FALSE(latch.OnPowerEnabled(true));
  EXPECT_TRUE(latch.IsFaultLatched());
}

TEST(MotorReadinessRecoveryTest, LatchedFaultRequiresExplicitRecoveryDisarmGate)
{
  lekiwi_base_controller::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnFreshEncoderSample();
  latch.OnEncoderTimeout();

  EXPECT_FALSE(lekiwi_base_controller::IsMotorPowerEnableAllowed(latch, false));
  latch.OnPowerDisabled();
  EXPECT_TRUE(lekiwi_base_controller::IsMotorPowerEnableAllowed(latch, true));
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  EXPECT_FALSE(latch.IsReady(true, true, true));
  latch.OnFreshEncoderSample();
  EXPECT_TRUE(latch.IsReady(true, true, true));
}

TEST(MotorReadinessLatchTest, DisconnectedOrNonHardwareBackendNeverReportsReady)
{
  lekiwi_base_controller::MotorReadinessLatch latch;

  EXPECT_FALSE(latch.OnPowerEnabled(false));
  EXPECT_FALSE(latch.IsReady(true, false, true));

  latch.OnPowerDisabled();
  ASSERT_TRUE(latch.OnPowerEnabled(true));
  latch.OnFreshEncoderSample();
  EXPECT_FALSE(latch.IsReady(false, true, true));
  EXPECT_FALSE(latch.IsReady(true, true, false));
}
