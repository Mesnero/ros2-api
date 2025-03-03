cmake_minimum_required(VERSION 3.8)
project(publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_api_msgs REQUIRED)
find_package(types_api REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create an INTERFACE library for a header-only package.
add_library(publisher INTERFACE)

# Specify include directories for the header-only library.
target_include_directories(publisher INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export usage requirements (dependencies) for consumers of this package.
ament_target_dependencies(publisher INTERFACE
  rclcpp
  trajectory_msgs
  std_msgs
  ros2_api_msgs
  types_api
  sensor_msgs
)

# Install the header files.
install(
  DIRECTORY include/ros2_api/publisher
  DESTINATION include/ros2_api
)

# Export the include directories.
ament_export_include_directories(include)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_publisher test/test_publisher.cpp)
  if(TARGET test_publisher)
    target_include_directories(test_publisher PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_publisher publisher)
  endif()
endif()

ament_package()
