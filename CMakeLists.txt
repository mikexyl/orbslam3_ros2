cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

install(TARGETS stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

install(FILES ${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt.tar.gz
              ${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt
  DESTINATION share/${PROJECT_NAME}/vocab)

ament_package()

