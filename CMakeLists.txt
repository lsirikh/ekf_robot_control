cmake_minimum_required(VERSION 3.5)
project(ekf_robot_control)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable for main_node
add_executable(main_node
  src/main.cpp
  src/interface.cpp
  src/extendedKalmanFilter.cpp
  src/utilities.cpp
)

# Add executable for setMeasurementCovariance_node
add_executable(set_measurement_covariance_node
  src/setMeasurementCovariance.cpp
)

ament_target_dependencies(main_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
)

ament_target_dependencies(set_measurement_covariance_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
)

# Install executables
install(TARGETS main_node set_measurement_covariance_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)


ament_package()
