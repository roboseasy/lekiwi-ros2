cmake_minimum_required(VERSION 3.8)
project(lekiwi_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# add the executable
add_executable(lekiwi_imu_node
  src/lekiwi_imu_node.cpp
  src/bmi160_driver.cpp
  src/kalman_filter.cpp
  src/low_pass_filter.cpp
  src/one_euro_filter.cpp
)
target_compile_features(lekiwi_imu_node PUBLIC cxx_std_17)
target_include_directories(lekiwi_imu_node PRIVATE include)
target_link_libraries(lekiwi_imu_node Eigen3::Eigen)
ament_target_dependencies(lekiwi_imu_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# install targets
install(TARGETS lekiwi_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 필터는 노드와 따로 시험한다 (ROS 없이)
  ament_add_gtest(test_filters
    test/test_filters.cpp
    src/kalman_filter.cpp
    src/low_pass_filter.cpp
    src/one_euro_filter.cpp
  )
  target_compile_features(test_filters PUBLIC cxx_std_17)
  target_include_directories(test_filters PRIVATE include)
  target_link_libraries(test_filters Eigen3::Eigen)
endif()

ament_package()
