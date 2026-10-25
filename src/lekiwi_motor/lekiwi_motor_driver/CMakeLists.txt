cmake_minimum_required(VERSION 3.8)
project(lekiwi_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lekiwi_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# add the executable
add_executable(lekiwi_motor_driver_node
  src/lekiwi_motor_driver_node.cpp
  src/feetech_bus.cpp
)
target_compile_features(lekiwi_motor_driver_node PUBLIC cxx_std_17)
target_include_directories(lekiwi_motor_driver_node PRIVATE include)
ament_target_dependencies(lekiwi_motor_driver_node rclcpp lekiwi_msgs std_msgs std_srvs)

# install targets
install(TARGETS lekiwi_motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# 가상 Feetech 버스 (시험용). 다른 패키지의 통합 시험이 설치 경로로 가져다 쓴다.
install(
  FILES test/fake_sts_bus.py
  DESTINATION share/${PROJECT_NAME}/test_support
)

if(BUILD_TESTING)
  # 패킷 · 값 변환 (하드웨어 없음)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_feetech_protocol
    test/test_feetech_protocol.cpp
    src/feetech_bus.cpp
  )
  target_compile_features(test_feetech_protocol PUBLIC cxx_std_17)
  target_include_directories(test_feetech_protocol PRIVATE include)

  # 가상 Feetech 버스(PTY)에 노드를 붙여 바퀴 쓰기 · 안전 처리를 확인한다 (하드웨어 없음)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_write_pty test/test_wheel_write_pty.py
    ENV LEKIWI_MOTOR_DRIVER_EXE=$<TARGET_FILE:lekiwi_motor_driver_node>
    TIMEOUT 180
  )
endif()

ament_package()
