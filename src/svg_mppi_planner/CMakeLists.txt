cmake_minimum_required(VERSION 3.5)
project(svg_mppi_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 -> C++17
if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(include/)

add_executable(svg_mppi_planner_node 
  src/svg_mppi_planner_node.cpp
  src/svg_mppi_planner_ros.cpp
  src/svg_mppi.cpp
)

ament_target_dependencies(svg_mppi_planner_node
  rclcpp
  grid_map_ros
)

target_include_directories(svg_mppi_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add OpenMP to target
if(OpenMP_CXX_FOUND)
  target_link_libraries(svg_mppi_planner_node OpenMP::OpenMP_CXX)
endif()


install(TARGETS svg_mppi_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
