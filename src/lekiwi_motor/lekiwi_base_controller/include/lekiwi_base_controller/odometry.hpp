#ifndef LEKIWI_BASE_CONTROLLER__ODOMETRY_HPP_
#define LEKIWI_BASE_CONTROLLER__ODOMETRY_HPP_

#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include "lekiwi_base_controller/omni_kinematics.hpp"

namespace lekiwi_base_controller
{

// odom 기준 평면 자세 (x, y, yaw) 적분
class Odometry
{
public:
  void Reset();
  // 몸체 속도 × dt 적분 (command 또는 speed 피드백)
  void Update(const geometry_msgs::msg::Twist & twist, double dt);
  // 바퀴 위치 증분에서 얻은 몸체 자세 증분 적용 (position 피드백)
  void UpdateDelta(const BodyPoseDelta & delta);

  nav_msgs::msg::Odometry ToMessage(
    const rclcpp::Time & stamp,
    const std::string & odom_frame,
    const std::string & base_frame,
    const geometry_msgs::msg::Twist & twist) const;

  geometry_msgs::msg::TransformStamped ToTransform(
    const rclcpp::Time & stamp,
    const std::string & odom_frame,
    const std::string & base_frame) const;

private:
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__ODOMETRY_HPP_
