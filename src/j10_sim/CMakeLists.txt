cmake_minimum_required(VERSION 3.8)
project(j10_sim)

find_package(ament_cmake REQUIRED)

# This package ships no compiled code — only worlds, parameters, launch files and scripts.

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/run_sitl.sh
    scripts/latency_probe.py
    scripts/link_loss_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # See the note in j10_mavlink/CMakeLists.txt — copyright/style linters are opted out of
  # until the workspace adopts a formatting pass.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
