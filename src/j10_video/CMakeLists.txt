cmake_minimum_required(VERSION 3.8)
project(j10_video)

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
find_package(sensor_msgs REQUIRED)
find_package(j10_interfaces REQUIRED)

# GStreamer comes from pkg-config, not from ament.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-rtp-1.0
)

set(J10_VIDEO_DEPENDENCIES rclcpp rclcpp_components sensor_msgs j10_interfaces)

# --- The node ------------------------------------------------------------------------------
# rtp_clock.hpp and stream_health.hpp are header-only and free of both ROS and GStreamer,
# so the tests below build and run without either.

add_library(j10_video_components SHARED src/video_receiver_node.cpp)
target_include_directories(j10_video_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(j10_video_components PkgConfig::GST)
ament_target_dependencies(j10_video_components ${J10_VIDEO_DEPENDENCIES})

rclcpp_components_register_nodes(j10_video_components "j10_video::VideoReceiverNode")

add_executable(video_receiver_node src/video_receiver_main.cpp)
target_link_libraries(video_receiver_node j10_video_components)
ament_target_dependencies(video_receiver_node ${J10_VIDEO_DEPENDENCIES})

# --- Install -------------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS j10_video_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS video_receiver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- Tests ---------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Same opt-out as the rest of the workspace -- see the note in j10_mavlink/CMakeLists.txt.
  # The repository carries no license headers and no uncrustify pass has been run, so these
  # three would be permanently red, which trains people to ignore CI. Re-enable them across
  # the whole workspace at once when a formatting pass is adopted, not package by package.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Neither test links ROS or GStreamer -- both headers are pure.
  ament_add_gtest(test_rtp_clock test/test_rtp_clock.cpp)
  target_include_directories(test_rtp_clock PRIVATE include)

  ament_add_gtest(test_stream_health test/test_stream_health.cpp)
  target_include_directories(test_stream_health PRIVATE include)
endif()

ament_package()
