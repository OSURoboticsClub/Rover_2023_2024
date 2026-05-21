cmake_minimum_required(VERSION 3.8)
project(rover_arm_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

#add_library(joy_to_servo SHARED src/joy_to_servo.cpp)

#target_include_directories(joy_to_servo PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SwitchController.srv"
  "srv/RelativeMoveSrv.srv"
  "action/RelativeMove.action"
  "action/GripperControl.action"
  "action/AutonTyping.action"
  DEPENDENCIES geometry_msgs
)

#ament_target_dependencies(
#  joy_to_servo
#  rclcpp
#  rclcpp_components
#  geometry_msgs
#  control_msgs
#  moveit_msgs
#  std_srvs
#  sensor_msgs
#)

#ament_export_dependencies(
#  rosidl_default_runtime
#  rclcpp_components
#)
# ament_package()


#rclcpp_components_register_node(joy_to_servo
#  PLUGIN "moveit_servo::JoyToServoPub"
#  EXECUTABLE joy_to_servo_node
#)

# Export the library target
#ament_export_targets(export_joy_to_servo_component)

# Install the shared library
#install(TARGETS joy_to_servo
#        EXPORT export_joy_to_servo_component
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME DESTINATION bin
#)


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
