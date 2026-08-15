cmake_minimum_required(VERSION 3.8)
project(j10_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavIntent.msg"
  "msg/SafetyStatus.msg"
  "msg/VehicleState.msg"
  "msg/StreamStatus.msg"
  "msg/LatencyReport.msg"
  "srv/SetMissionState.srv"
  "srv/SetInstruction.srv"
  "srv/ArmDisarm.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
