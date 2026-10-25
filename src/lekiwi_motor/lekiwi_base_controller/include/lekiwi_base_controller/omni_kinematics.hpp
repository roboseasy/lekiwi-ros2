#ifndef LEKIWI_BASE_CONTROLLER__OMNI_KINEMATICS_HPP_
#define LEKIWI_BASE_CONTROLLER__OMNI_KINEMATICS_HPP_

#include <algorithm>
#include <cmath>

namespace lekiwi_base_controller
{

// 바퀴 속도 [rad/s]. 순서는 모델 관절 left, back, right
struct WheelCommand
{
  double left{0.0};
  double back{0.0};
  double right{0.0};
};

// 몸체 속도 (base_footprint 기준)
struct BodyTwist
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

struct WheelPositionDelta
{
  double left{0.0};   // rad
  double back{0.0};   // rad
  double right{0.0};  // rad
};

struct BodyPoseDelta
{
  double dx{0.0};    // 몸체 기준 m
  double dy{0.0};    // 몸체 기준 m
  double dyaw{0.0};  // rad
};

// 3륜 옴니 휠 기구학.
// kinematics_frame_*: 원래 모터 기구학 원점을 현재 base_footprint에서 본 자세
// (lekiwi_description/config/base_frame.yaml과 같은 값)
class OmniKinematics
{
public:
  OmniKinematics() = default;
  OmniKinematics(
    double wheel_radius, double base_radius, double max_wheel_speed,
    double kinematics_frame_x = 0.0, double kinematics_frame_y = 0.0,
    double kinematics_frame_yaw = 0.0);

  // 가장 빠른 바퀴가 max_wheel_speed를 넘지 않도록 세 축을 같은 비율로 줄인다
  BodyTwist ScaleToWheelEnvelope(const BodyTwist & twist) const;
  WheelCommand ToWheelSpeeds(double linear_x, double linear_y, double angular_z) const;
  BodyTwist ToBodyTwist(const WheelCommand & command) const;
  BodyPoseDelta ToBodyDelta(const WheelPositionDelta & delta) const;

private:
  BodyTwist ToKinematicsFrame(const BodyTwist & twist) const;
  BodyTwist FromKinematicsFrame(const BodyTwist & twist) const;

  double wheel_radius_{0.05};
  double base_radius_{0.13647};
  double max_wheel_speed_{20.0};
  double kinematics_frame_x_{0.0};
  double kinematics_frame_y_{0.0};
  double kinematics_frame_yaw_{0.0};
};

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__OMNI_KINEMATICS_HPP_
