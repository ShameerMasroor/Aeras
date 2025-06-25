cmake_minimum_required(VERSION 3.8)
project(apf_local)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(apf_local_planner src/apf_local_planner.cpp)
ament_target_dependencies(apf_local_planner
  rclcpp
  geometry_msgs
  nav_msgs
  octomap_msgs
  octomap
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS apf_local_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
