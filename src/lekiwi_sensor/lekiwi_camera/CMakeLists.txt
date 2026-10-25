cmake_minimum_required(VERSION 3.8)
project(lekiwi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# OpenCV는 rotation을 쓸 때 JPEG 디코딩 · 회전 · 재압축에만 쓴다.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# add the executable
add_executable(lekiwi_camera_node
  src/lekiwi_camera_node.cpp
  src/v4l2_capture.cpp
)
target_compile_features(lekiwi_camera_node PUBLIC cxx_std_17)
target_include_directories(lekiwi_camera_node PRIVATE include)
target_include_directories(lekiwi_camera_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lekiwi_camera_node ${OpenCV_LIBS})
ament_target_dependencies(lekiwi_camera_node rclcpp sensor_msgs)

# install targets
install(TARGETS lekiwi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
