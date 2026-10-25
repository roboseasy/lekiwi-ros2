cmake_minimum_required(VERSION 3.8)
project(lekiwi_scan_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (YDLidar SDK는 쓰지 않는다. 스캔 토픽만 받는다)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# add the executable
add_executable(lekiwi_scan_filter_node
  src/lekiwi_scan_filter_node.cpp
  src/scan_filter.cpp
)
target_compile_features(lekiwi_scan_filter_node PUBLIC cxx_std_17)
target_include_directories(lekiwi_scan_filter_node PRIVATE include)
ament_target_dependencies(lekiwi_scan_filter_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

# install targets
install(TARGETS lekiwi_scan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # 필터 함수만 검사한다 (ROS 통신 · TF 없이)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_filter
    test/test_scan_filter.cpp
    src/scan_filter.cpp
  )
  target_compile_features(test_scan_filter PUBLIC cxx_std_17)
  target_include_directories(test_scan_filter PRIVATE include)
  ament_target_dependencies(test_scan_filter sensor_msgs)
endif()

ament_package()
