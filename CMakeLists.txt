cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)

# Other dependencies
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(random_forest src/main.cpp src/RandomMapSensing.cpp)
ament_target_dependencies(random_forest rclcpp std_msgs geometry_msgs pcl_conversions nav_msgs)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  random_forest
  DESTINATION lib/${PROJECT_NAME})

ament_package()
