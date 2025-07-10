cmake_minimum_required(VERSION 3.8)
project(clr_dynamic_sim_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(dex_ivr_interfaces REQUIRED)
find_package(color_blob_centroid REQUIRED)

set(dependencies
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  tf2
  dex_ivr_interfaces
  color_blob_centroid
)

add_executable(run_demo src/run_demo.cpp)
ament_target_dependencies(run_demo ${dependencies})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS run_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
