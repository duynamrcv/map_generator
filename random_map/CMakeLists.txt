cmake_minimum_required(VERSION 3.8)
project(random_map)

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

add_executable(random_forest_node src/main.cpp src/RandomMapSensing.cpp)
ament_target_dependencies(random_forest_node rclcpp std_msgs geometry_msgs pcl_conversions nav_msgs)

add_library(${PROJECT_NAME} SHARED src/RandomMapSensing.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs std_msgs pcl_conversions nav_msgs)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

install(DIRECTORY
  include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  random_forest_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
