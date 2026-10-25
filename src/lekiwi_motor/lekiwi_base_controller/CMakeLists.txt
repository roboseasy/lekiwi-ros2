cmake_minimum_required(VERSION 3.8)
project(lekiwi_base_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lekiwi_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  lekiwi_msgs
)

# 노드와 테스트가 함께 쓰는 라이브러리
add_library(lekiwi_base_controller_core STATIC
  src/base_controller.cpp
  src/driver_motor_interface.cpp
  src/lekiwi_base_controller_node.cpp
  src/motor_readiness.cpp
  src/motor_safety_coordinator.cpp
  src/odometry.cpp
  src/omni_kinematics.cpp
  src/wheel_feedback_estimator.cpp
)
target_compile_features(lekiwi_base_controller_core PUBLIC cxx_std_17)
target_include_directories(lekiwi_base_controller_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
set_target_properties(lekiwi_base_controller_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(lekiwi_base_controller_core ${dependencies})

# add the executable
add_executable(lekiwi_base_controller_node src/lekiwi_base_controller_main.cpp)
target_link_libraries(lekiwi_base_controller_node lekiwi_base_controller_core)

# install targets
install(TARGETS lekiwi_base_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 순수 계산 · 상태기계
  ament_add_gtest(test_omni_kinematics test/test_omni_kinematics.cpp)
  target_link_libraries(test_omni_kinematics lekiwi_base_controller_core)

  ament_add_gtest(test_motor_readiness test/test_motor_readiness.cpp)
  target_link_libraries(test_motor_readiness lekiwi_base_controller_core)

  ament_add_gtest(test_motor_safety_coordinator test/test_motor_safety_coordinator.cpp)
  target_link_libraries(test_motor_safety_coordinator lekiwi_base_controller_core)

  ament_add_gtest(test_base_motor_interface_contract test/test_base_motor_interface_contract.cpp)
  target_link_libraries(test_base_motor_interface_contract lekiwi_base_controller_core)

  ament_add_gtest(test_wheel_feedback_estimator test/test_wheel_feedback_estimator.cpp)
  target_link_libraries(test_wheel_feedback_estimator lekiwi_base_controller_core)

  ament_add_gtest(test_base_node_smoke test/test_base_node_smoke.cpp)
  target_link_libraries(test_base_node_smoke lekiwi_base_controller_core)
  target_compile_definitions(test_base_node_smoke PRIVATE
    LEKIWI_BASE_CONTROLLER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # 가짜 모터 드라이버와 함께 도는 ROS 통합 테스트
  # 로봇 · 다른 PC와 섞이지 않도록 이 PC 안에서만 discovery 한다
  ament_add_gtest(test_driver_integration test/test_driver_integration.cpp
    TIMEOUT 120
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
  target_link_libraries(test_driver_integration lekiwi_base_controller_core)
endif()

ament_package()
