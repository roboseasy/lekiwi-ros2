cmake_minimum_required(VERSION 3.8)
project(lekiwi_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmCommand.msg"
  "msg/ArmRegisters.msg"
  "msg/LerobotState.msg"
  "msg/MotorState.msg"
  "msg/MotorStateArray.msg"
  "msg/WheelCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
