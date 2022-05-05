cmake_minimum_required(VERSION 3.5)
project(create_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/BatteryState.msg"
  "msg/Drive.msg"
  "msg/RawTurtlebotSensorState.msg"
  "msg/RoombaSensorState.msg"
  "msg/Turtle.msg"
  "msg/TurtlebotSensorState.msg"
  "msg/ScanAngle.msg"
)

set(srv_files
  "srv/SetDigitalOutputs.srv"
  "srv/SetTurtlebotMode.srv"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
