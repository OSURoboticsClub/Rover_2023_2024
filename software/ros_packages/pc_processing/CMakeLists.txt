cmake_minimum_required(VERSION 3.8)
project(pc_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#Not sure what this does
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# C++ packages need rclcpp.  Add in the other packages we need here, too.
find_package(PCL 1.3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#Generate the interfaces:
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetObjects.srv"
  #DEPENDENCIES buitltin_interfaces
)



# We need this to make the PCL stuff work.
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add in the executables.
add_executable(pc_filter src/pc_filter.cpp)
target_link_libraries(pc_filter ${PCL_LIBRARIES})
ament_target_dependencies(pc_filter rclcpp sensor_msgs pcl_ros pcl_conversions tf2_ros tf2_sensor_msgs geometry_msgs ) #Include ros, sensor_msgs, pcl libraries as dependencies

add_executable(plane_fit src/plane_fit.cpp)
target_link_libraries(plane_fit ${PCL_LIBRARIES})
ament_target_dependencies(plane_fit rclcpp sensor_msgs pcl_ros pcl_conversions visualization_msgs shape_msgs)

add_executable(count_objects src/count_objects.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(count_objects "${cpp_typesupport_target}")
target_link_libraries(count_objects ${PCL_LIBRARIES})
ament_target_dependencies(count_objects rclcpp sensor_msgs visualization_msgs std_msgs pcl_ros pcl_conversions)

#Also add the node execs to the install targets
install(TARGETS 
  
  pc_filter
  plane_fit
  count_objects

  DESTINATION lib/${PROJECT_NAME})


# Install the include files so that other pacakges can find them.
install(DIRECTORY
  include

  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	config
	DESTINATION share/${PROJECT_NAME}
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

ament_package()
