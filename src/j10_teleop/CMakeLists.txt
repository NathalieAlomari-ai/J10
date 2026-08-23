cmake_minimum_required(VERSION 3.8)
project(j10_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(J10_TELEOP_DEPENDENCIES rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs)

# --- The node ------------------------------------------------------------------------------
# joystick_mapping.hpp is header-only and ROS-free, so the test below links no ROS runtime.

add_library(j10_teleop_components SHARED src/teleop_override_node.cpp)
target_include_directories(j10_teleop_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(j10_teleop_components ${J10_TELEOP_DEPENDENCIES})

rclcpp_components_register_nodes(j10_teleop_components "j10_teleop::TeleopOverrideNode")

add_executable(teleop_override_node src/teleop_override_main.cpp)
target_link_libraries(teleop_override_node j10_teleop_components)
ament_target_dependencies(teleop_override_node ${J10_TELEOP_DEPENDENCIES})

# --- Install -------------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS j10_teleop_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS teleop_override_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- Tests ---------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Links nothing but the header -- the mapping is pure and needs no ROS to test.
  ament_add_gtest(test_joystick_mapping test/test_joystick_mapping.cpp)
  target_include_directories(test_joystick_mapping PRIVATE include)
endif()

ament_package()
