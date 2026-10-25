cmake_minimum_required(VERSION 3.8)
project(lekiwi_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# YDLidar SDK (/usr/local에 설치). SDK 설정 파일이 라이브러리 경로를 비워 두므로
# 설정 파일 위치(<prefix>/lib/cmake/ydlidar_sdk)에서 <prefix>/lib를 직접 찾는다.
find_package(ydlidar_sdk REQUIRED)
find_library(YDLIDAR_SDK_LIBRARY ydlidar_sdk
  HINTS "${ydlidar_sdk_DIR}/../.."
  REQUIRED)

# add the executable
add_executable(lekiwi_lidar_node
  src/lekiwi_lidar_node.cpp
  src/ydlidar_driver.cpp
)
target_compile_features(lekiwi_lidar_node PUBLIC cxx_std_17)
target_include_directories(lekiwi_lidar_node PRIVATE include)
# SDK 헤더에서 나오는 경고는 이 패키지 경고로 세지 않는다.
target_include_directories(lekiwi_lidar_node SYSTEM PRIVATE ${YDLIDAR_SDK_INCLUDE_DIRS})
target_link_libraries(lekiwi_lidar_node ${YDLIDAR_SDK_LIBRARY} pthread rt)
ament_target_dependencies(lekiwi_lidar_node rclcpp sensor_msgs)

# install targets
install(TARGETS lekiwi_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
