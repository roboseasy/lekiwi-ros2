#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lekiwi_base_controller/lekiwi_base_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<lekiwi_base_controller::LekiwiBaseControllerNode>();
    // 서비스 · 타이머 안에서 드라이버 토크 서비스를 동기 호출하므로 여러 스레드로 돌린다
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      lekiwi_base_controller::LekiwiBaseControllerNode::executor_thread_count);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lekiwi_base_controller_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
