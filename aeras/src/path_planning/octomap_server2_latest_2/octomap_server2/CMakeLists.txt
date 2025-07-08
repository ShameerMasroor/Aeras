# cmake_minimum_required(VERSION 3.5)
# project(octomap_server2)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(PCL 1.10 REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(OpenMP REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(message_filters REQUIRED)
# find_package(octomap REQUIRED)
# find_package(octomap_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(rclcpp_components REQUIRED)

# link_directories(${PCL_LIBRARY_DIRS})
# include_directories(${PCL_INCLUDE_DIRS})
# include_directories(include)

# add_library(octomap_server2 SHARED
#   src/octomap_server.cpp
#   src/transforms.cpp
#   src/conversions.cpp
#   )

# ament_target_dependencies(octomap_server2
#   rclcpp
#   PCL
#   pcl_conversions
#   sensor_msgs
#   std_msgs
#   nav_msgs
#   visualization_msgs
#   geometry_msgs
#   std_srvs
#   octomap
#   octomap_msgs
#   message_filters
#   tf2_ros
#   tf2_msgs
#   tf2
#   tf2_geometry_msgs
#   rclcpp_components
#   )

# target_link_libraries(octomap_server2 ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

# rclcpp_components_register_node(octomap_server2
#   PLUGIN "octomap_server::OctomapServer"
#   EXECUTABLE octomap_server)

# install(TARGETS
#   octomap_server2
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
#   )

# ament_package()


cmake_minimum_required(VERSION 3.5)
project(octomap_server2)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

# Shared library for OctomapServer
add_library(octomap_server2 SHARED
  src/octomap_server.cpp
  src/transforms.cpp
  src/conversions.cpp
  src/load_octomap_node
)

# Separate executable for loading OctoMap
add_executable(load_octomap_node
  src/load_octomap_node.cpp
)

add_executable(octomap_analyzer
  src/octomap_analyzer.cpp
)

# Dependencies for OctomapServer library
ament_target_dependencies(octomap_server2
  rclcpp
  rclcpp_components
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
  octomap
  octomap_msgs
  message_filters
  tf2_ros
  tf2_msgs
  tf2
  tf2_geometry_msgs
)

# Dependencies for load_octomap_node executable
ament_target_dependencies(load_octomap_node
  rclcpp
  rclcpp_components
  octomap
  octomap_msgs
)

ament_target_dependencies(octomap_analyzer
  rclcpp
  octomap
  octomap_msgs
)

# Link OctomapServer library to PCL and OctoMap libraries
target_link_libraries(octomap_server2 ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries(load_octomap_node octomap_server2 ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

# Register OctomapServer as a component
rclcpp_components_register_node(octomap_server2
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server)

# Install targets
# install(TARGETS
#   octomap_server2
#   load_octomap_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
install(TARGETS
  octomap_server2
  load_octomap_node
  octomap_analyzer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
