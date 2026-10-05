cmake_minimum_required(VERSION 3.8)
project(j10_control)

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
find_package(j10_interfaces REQUIRED)

set(J10_CONTROL_DEPENDENCIES rclcpp rclcpp_components geometry_msgs j10_interfaces)

# --- The shaper --------------------------------------------------------------------------
# Depends only on geometry_msgs so the tests link no ROS runtime.

add_library(j10_motion_shaper SHARED src/motion_shaper.cpp)
target_include_directories(j10_motion_shaper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(j10_motion_shaper geometry_msgs)

# --- The node ----------------------------------------------------------------------------

add_library(j10_control_components SHARED src/motion_controller_node.cpp)
target_include_directories(j10_control_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(j10_control_components j10_motion_shaper)
ament_target_dependencies(j10_control_components ${J10_CONTROL_DEPENDENCIES})

rclcpp_components_register_nodes(j10_control_components "j10_control::MotionControllerNode")

add_executable(motion_controller_node src/motion_controller_main.cpp)
target_link_libraries(motion_controller_node j10_control_components j10_motion_shaper)
ament_target_dependencies(motion_controller_node ${J10_CONTROL_DEPENDENCIES})

# --- Install -----------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS j10_motion_shaper j10_control_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS motion_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- Test ---------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # See the note in j10_mavlink/CMakeLists.txt.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_shaper test/test_motion_shaper.cpp)
  if(TARGET test_motion_shaper)
    target_link_libraries(test_motion_shaper j10_motion_shaper)
    ament_target_dependencies(test_motion_shaper geometry_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(j10_motion_shaper j10_control_components)
ament_export_dependencies(${J10_CONTROL_DEPENDENCIES})

ament_package()
