cmake_minimum_required(VERSION 3.8)
project(j10_safety)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(j10_interfaces REQUIRED)

set(J10_SAFETY_DEPENDENCIES
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  std_srvs
  j10_interfaces
)

# --- The filter itself -------------------------------------------------------------------
# Separate from the node, and depending only on geometry_msgs, so the test binary below
# links no ROS runtime at all. Boundary rule 3: if the safety filter cannot be tested from
# a plain gtest binary in under a second, it is built wrong.

add_library(j10_safety_filter SHARED src/safety_filter.cpp)
target_include_directories(j10_safety_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(j10_safety_filter geometry_msgs)

# --- The node ----------------------------------------------------------------------------

add_library(j10_safety_components SHARED src/safety_filter_node.cpp)
target_include_directories(j10_safety_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(j10_safety_components j10_safety_filter)
ament_target_dependencies(j10_safety_components ${J10_SAFETY_DEPENDENCIES})

rclcpp_components_register_nodes(j10_safety_components "j10_safety::SafetyFilterNode")

add_executable(safety_filter_node src/safety_filter_main.cpp)
target_link_libraries(safety_filter_node j10_safety_components j10_safety_filter)
ament_target_dependencies(safety_filter_node ${J10_SAFETY_DEPENDENCIES})

# --- Install -----------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS j10_safety_filter j10_safety_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS safety_filter_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- Test ---------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # See the note in j10_mavlink/CMakeLists.txt -- style and copyright linters are opted
  # out of until the workspace adopts a formatting pass.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_filter test/test_safety_filter.cpp)
  if(TARGET test_safety_filter)
    target_link_libraries(test_safety_filter j10_safety_filter)
    ament_target_dependencies(test_safety_filter geometry_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(j10_safety_filter j10_safety_components)
ament_export_dependencies(${J10_SAFETY_DEPENDENCIES})

ament_package()
