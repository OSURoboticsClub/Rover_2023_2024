cmake_minimum_required(VERSION 3.5)
project(rover_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_camera_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV)
find_package(sensor_msgs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APPSINK REQUIRED gstreamer-app-1.0)

include_directories(
    ${GST_INCLUDE_DORS}
    ${GST_APPSINK_INCLUDE_DIRS}
)
link_directories(${GST_LIBRARY_DIRS})

add_executable(rover_camera src/rover_camera.cpp)
ament_target_dependencies(rover_camera rclcpp rover_camera_interface cv_bridge image_transport OpenCV sensor_msgs)

target_link_libraries(rover_camera
    ${GST_LIBRARIES}
    ${GST_APPSINK_LIBRARIES})

install(TARGETS
  rover_camera
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

