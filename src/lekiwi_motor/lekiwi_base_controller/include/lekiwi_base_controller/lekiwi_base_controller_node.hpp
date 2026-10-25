#ifndef LEKIWI_BASE_CONTROLLER__LEKIWI_BASE_CONTROLLER_NODE_HPP_
#define LEKIWI_BASE_CONTROLLER__LEKIWI_BASE_CONTROLLER_NODE_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "lekiwi_base_controller/base_controller.hpp"
#include "lekiwi_base_controller/motor_safety_coordinator.hpp"
#include "lekiwi_base_controller/odometry.hpp"
#include "lekiwi_base_controller/omni_kinematics.hpp"

namespace lekiwi_base_controller
{

// cmd_vel → 바퀴 속도, 바퀴 엔코더 → odom, 모터 안전 상태기계.
//   구독: cmd_vel_topic (geometry_msgs/Twist)
//   발행: odom_topic, wheel_joint_states_topic, motor_ready_topic, TF odom → base_footprint
//   서비스: motor_power_service (SetBool), reset_odometry_service (Trigger)
// 제어 타이머 · cmd_vel · 두 서비스는 한 상호배타 그룹(control_group_)에서 차례로 실행한다.
// MultiThreadedExecutor(executor_thread_count)로 돌려야 서비스 · 타이머 안에서 부르는
// 드라이버 토크 서비스 응답이 다른 스레드에서 처리된다.
class LekiwiBaseControllerNode : public rclcpp::Node
{
public:
  static constexpr std::size_t executor_thread_count = 4;

  explicit LekiwiBaseControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LekiwiBaseControllerNode() override;

private:
  // 파라미터 선언 · 로드
  BaseControllerParameters LoadParameters();
  void WarnAboutBackendSelection() const;

  // 시작
  void CreateMotorInterface();
  void StartMotorInterface();
  void CreateRosInterfaces();

  // 콜백
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void ControlTimerCallback();
  void ResetOdometryCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void MotorPowerCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void PublishMotorReady();
  bool IsRealMotorHardware() const;

  BaseControllerParameters params_;
  OmniKinematics kinematics_;
  Odometry odometry_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_update_time_;
  bool has_cmd_{false};
  std::array<double, WHEEL_COUNT> wheel_angles_{};

  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_ready_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odometry_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motor_power_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<BaseMotorInterface> motor_interface_;
  std::unique_ptr<MotorSafetyCoordinator> motor_safety_;
  std::optional<rclcpp::PreShutdownCallbackHandle> pre_shutdown_handle_;
};

}  // namespace lekiwi_base_controller

#endif  // LEKIWI_BASE_CONTROLLER__LEKIWI_BASE_CONTROLLER_NODE_HPP_
