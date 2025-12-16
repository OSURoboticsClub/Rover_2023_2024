cmake_minimum_required(VERSION 3.8)
project(joy_to_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(joy REQUIRED)
find_package(rover_arm_urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(joy_to_servo_node src/joy_to_servo_node.cpp)
add_executable(controller_switcher src/controller_switcher.cpp)
add_executable(manipulability src/manipulability.cpp)

target_include_directories(joy_to_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(manipulability PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_compile_features(joy_to_servo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joy_to_servo_node
  "moveit_servo"
  "joy"
  "rover_arm_urdf"
)
ament_target_dependencies(
  controller_switcher
  "rclcpp"
  "sensor_msgs"
  "controller_manager_msgs"
)
ament_target_dependencies(
  manipulability
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
)

install(TARGETS joy_to_servo_node controller_switcher manipulability
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
