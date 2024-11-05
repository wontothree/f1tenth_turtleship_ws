cmake_minimum_required(VERSION 3.5)
project(mppi_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(include/)

add_executable(mppi_planner_node
  src/mppi_planner_ros.cpp
  src/mppi.cpp
  src/mppi_planner_node.cpp
)

ament_target_dependencies(mppi_planner_node
  rclcpp
  grid_map_ros
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(mppi_planner_node OpenMP::OpenMP_CXX)
endif()

target_include_directories(mppi_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS mppi_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
