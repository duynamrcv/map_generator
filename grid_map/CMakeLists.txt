cmake_minimum_required(VERSION 3.8)
project(grid_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Generate the compile_commands.json file
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

# Other dependencies
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(grid_map src/main.cpp src/GridMap.cpp src/RayCaster.cpp)
ament_target_dependencies(grid_map rclcpp geometry_msgs pcl_conversions nav_msgs visualization_msgs)

install(TARGETS
  grid_map
  DESTINATION lib/${PROJECT_NAME})

ament_package()
