// IMU 필터 시험. 기대값은 2026-10-18 정리 전 필터 구현으로 같은 입력을 거른 출력이다
// (필터를 다시 쓰면서 동작이 바뀌지 않았는지 확인한다).

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include "lekiwi_imu/kalman_filter.hpp"
#include "lekiwi_imu/low_pass_filter.hpp"
#include "lekiwi_imu/one_euro_filter.hpp"

namespace lekiwi_imu
{
namespace
{

const std::vector<double> kLowPassExpected = {
  1,
  1.2999999999999998,
  2.1099999999999999,
  2.3769999999999998,
  3.2754000000000003,
  -0.14492000000000016,
  -0.028984000000000027,
};

const std::vector<double> kOneEuroExpected = {
  0.0082705667702732549,
  0.011643566680811245,
  0.017703219241116432,
  0.024833881241366651,
  0.03488053172646776,
  0.043621154120996333,
  0.05791690441479682,
  0.074653194734001907,
  0.093479687864072167,
  0.11469673179066309,
  0.22604512123945139,
  0.3679251825734775,
  0.50003945253807802,
  0.6357055197913748,
  0.75198179179732039,
  0.84119394051518559,
  0.93625193836163734,
  1.0160275387788837,
  1.0883838274648774,
  1.1502453337828831,
  1.1979367695686283,
  1.3294611055279164,
  1.3534579799766528,
  1.375006906482624,
  1.39811134350071,
  1.4155007746375903,
  1.4366243851231622,
  1.4604553673668568,
  1.4789943893463584,
  1.5004888429376939,
};

// 1, 5, 11번째 단계의 상태 6개씩
const std::vector<double> kKalmanExpected = {
  0.010365139837867562,
  0.08719571680177704,
  0.21268256991893378,
  0.28987828672071086,
  0.40768256991893381,
  0.49219571680177704,
  0.057559577729761724,
  0.042871630562946156,
  0.2569537800446029,
  0.24278960105514022,
  0.45764160723756775,
  0.44218380336998137,
  0.15185434402101755,
  -0.051211522617314806,
  0.35093110316211285,
  0.14870649488938645,
  0.55193632651431646,
  0.34778325403048166,
};

constexpr double kTolerance = 1e-12;
constexpr double kPi = 3.14159265358979323846;

TEST(LowPassFilterTest, MatchesReferenceIncludingAlphaChange)
{
  LowPassFilter filter(0.3);
  const double inputs[] = {1.0, 2.0, 4.0, 3.0, 3.5, -1.0, 0.0};
  for (std::size_t i = 0; i < kLowPassExpected.size(); ++i) {
    const double output = i < 4 ? filter.Filter(inputs[i]) : filter.Filter(inputs[i], 0.8);
    EXPECT_NEAR(output, kLowPassExpected[i], kTolerance) << "sample " << i;
  }
}

TEST(LowPassFilterTest, FirstInputPassesThroughAndResetForgets)
{
  LowPassFilter filter(0.1);
  EXPECT_FALSE(filter.HasOutput());
  EXPECT_DOUBLE_EQ(filter.Filter(5.0), 5.0);
  EXPECT_TRUE(filter.HasOutput());
  EXPECT_DOUBLE_EQ(filter.LastOutput(), 5.0);
  filter.Reset();
  EXPECT_FALSE(filter.HasOutput());
  EXPECT_DOUBLE_EQ(filter.Filter(-2.0), -2.0);
}

TEST(LowPassFilterTest, RejectsAlphaOutsideZeroToOne)
{
  EXPECT_THROW(LowPassFilter(0.0), std::invalid_argument);
  EXPECT_THROW(LowPassFilter(1.5), std::invalid_argument);
  EXPECT_THROW(LowPassFilter(std::nan("")), std::invalid_argument);
  LowPassFilter filter;
  EXPECT_THROW(filter.Filter(1.0, -0.1), std::invalid_argument);
}

// 100 Hz 시작, 흔들리는 시각, 0.1 s에 계단, 20번째는 거꾸로 간 시각, 25번째부터는 시각 없음
TEST(OneEuroFilterTest, MatchesReferenceWithJitterStepAndMissingStamps)
{
  OneEuroFilter filter;
  filter.Configure({100.0, 1.0, 0.1, 1.0});
  double stamp = 0.0;
  for (int i = 0; i < static_cast<int>(kOneEuroExpected.size()); ++i) {
    stamp += 0.01 + 0.0005 * ((i * 3) % 5 - 2);
    const double value = std::sin(2.0 * kPi * 0.5 * stamp) + (i >= 10 ? 0.8 : 0.0)
      + 0.01 * ((i * 7) % 5 - 2);
    std::optional<double> used_stamp = stamp;
    if (i == 20) {
      used_stamp = stamp - 0.05;
    }
    if (i >= 25) {
      used_stamp.reset();
    }
    EXPECT_NEAR(filter.Filter(value, used_stamp), kOneEuroExpected[i], kTolerance) << "sample " << i;
  }
}

TEST(OneEuroFilterTest, ConfigureClearsHistory)
{
  OneEuroFilter filter;
  filter.Configure({100.0, 1.0, 0.1, 1.0});
  filter.Filter(10.0, 0.0);
  filter.Filter(10.5, 0.01);
  filter.Configure({100.0, 1.0, 0.1, 1.0});
  EXPECT_DOUBLE_EQ(filter.Filter(-3.0, 1.0), -3.0);   // 첫 입력은 그대로
}

TEST(OneEuroFilterTest, RejectsNonPositiveRatesAndCutoffs)
{
  OneEuroFilter filter;
  EXPECT_THROW(filter.Configure({0.0, 1.0, 0.1, 1.0}), std::invalid_argument);
  EXPECT_THROW(filter.Configure({100.0, 0.0, 0.1, 1.0}), std::invalid_argument);
  EXPECT_THROW(filter.Configure({100.0, 1.0, 0.1, -1.0}), std::invalid_argument);
}

// 6상태, 전이 · 관측 = I, 노드 기본값 Q 5e-6 · R 1e-4, 첫 측정으로 상태를 맞춘다
TEST(KalmanFilterTest, MatchesReferenceWithIdentityModel)
{
  KalmanFilter filter;
  filter.Initialize(6, 6, 0.000005, 0.0001);
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  std::size_t checked = 0;
  for (int step = 0; step < 12; ++step) {
    Eigen::VectorXd measurement(6);
    for (int axis = 0; axis < 6; ++axis) {
      measurement(axis) = 0.1 * axis + 0.02 * step * (axis % 2 == 0 ? 1.0 : -1.0)
        + 0.005 * ((step * 5 + axis) % 3 - 1);
    }
    if (step == 0) {
      filter.SetState(measurement);
    }
    filter.Predict(identity);
    filter.Correct(measurement, identity);
    if (step == 1 || step == 5 || step == 11) {
      for (int axis = 0; axis < 6; ++axis) {
        EXPECT_NEAR(filter.State()(axis), kKalmanExpected[checked++], kTolerance)
          << "step " << step << " axis " << axis;
      }
    }
  }
  EXPECT_EQ(checked, kKalmanExpected.size());
}

TEST(KalmanFilterTest, StartsAtZeroAndMovesTowardMeasurement)
{
  KalmanFilter filter;
  filter.Initialize(2, 2, 0.01, 0.01);
  EXPECT_TRUE(filter.State().isZero());
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd measurement(2);
  measurement << 1.0, -1.0;
  filter.Predict(identity);
  filter.Correct(measurement, identity);
  EXPECT_GT(filter.State()(0), 0.5);
  EXPECT_LT(filter.State()(0), 1.0);
  EXPECT_LT(filter.State()(1), -0.5);
}

}  // namespace
}  // namespace lekiwi_imu
