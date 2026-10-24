cmake_minimum_required(VERSION 3.8)
project(rover2_drill_urc26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(PROGRAMS
  rover2_drill_urc26/linear_actuator_control.py
  DESTINATION lib/${PROJECT_NAME}
  # TODO: remove rename
  RENAME linear_actuator_control
)
# TODO: remove following install block
install(PROGRAMS
  rover2_drill_urc26/linear_actuator_command.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME linear_actuator_command
)
install(PROGRAMS
  rover2_drill_urc26/drill_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drill_control
)
install(PROGRAMS
  rover2_drill_urc26/drill_cap_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drill_cap_control
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
