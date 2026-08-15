cmake_minimum_required(VERSION 3.8)
project(j10_mavlink)

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
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(j10_interfaces REQUIRED)

set(J10_MAVLINK_DEPENDENCIES
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_srvs
  mavros_msgs
  j10_interfaces
)

# --- Components -------------------------------------------------------------------------
# Both nodes are built as composable components so they can share one process with
# use_intra_process_comms:=true (see docs/ARCHITECTURE.md section 4).

add_library(j10_mavlink_components SHARED
  src/mavlink_bridge_node.cpp
  src/vehicle_state_node.cpp
)
target_include_directories(j10_mavlink_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(j10_mavlink_components ${J10_MAVLINK_DEPENDENCIES})

# Registered for container loading only. No generated executable is requested here: the
# generated main spins a SingleThreadedExecutor, and mavlink_bridge_node blocks on MAVROS
# service calls inside its own service callbacks. Load these into component_container_mt.
rclcpp_components_register_nodes(j10_mavlink_components
  "j10_mavlink::MavlinkBridgeNode"
  "j10_mavlink::VehicleStateNode"
)

# --- Standalone executables -------------------------------------------------------------

add_executable(mavlink_bridge_node src/mavlink_bridge_main.cpp)
target_link_libraries(mavlink_bridge_node j10_mavlink_components)
ament_target_dependencies(mavlink_bridge_node ${J10_MAVLINK_DEPENDENCIES})

add_executable(vehicle_state_node src/vehicle_state_main.cpp)
target_link_libraries(vehicle_state_node j10_mavlink_components)
ament_target_dependencies(vehicle_state_node ${J10_MAVLINK_DEPENDENCIES})

# --- Install ------------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS j10_mavlink_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS mavlink_bridge_node vehicle_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- Test ---------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Style and copyright linters are opted out of for now: the repository carries no license
  # headers and no uncrustify pass has been run against these sources. Marking them FOUND
  # is the supported way to skip a linter. Re-enable them one at a time as the workspace
  # adopts a formatting pass — a permanently red lint job trains people to ignore CI.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_conventions test/test_frame_conventions.cpp)
  if(TARGET test_frame_conventions)
    target_include_directories(test_frame_conventions PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(test_frame_conventions geometry_msgs mavros_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(j10_mavlink_components)
ament_export_dependencies(${J10_MAVLINK_DEPENDENCIES})

ament_package()
