cmake_minimum_required(VERSION 3.5)
project(local_costmap_generator)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

include_directories(include)

add_executable(local_costmap_generator_node 
  src/local_costmap_generator_node.cpp
  src/local_costmap_generator.cpp
)

ament_target_dependencies(local_costmap_generator_node
  rclcpp
  sensor_msgs
  tf2
  laser_geometry
  pcl_conversions
  pcl_ros
  grid_map_ros
  grid_map_msgs
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(local_costmap_generator_node PUBLIC OpenMP::OpenMP_CXX)
endif()

target_include_directories(local_costmap_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS local_costmap_generator_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
