cmake_minimum_required(VERSION 3.8)
project(protocol_base VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# install base headers
install(DIRECTORY include/ros2_api/protocol_base
  DESTINATION include/ros2_api)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_export_include_directories(
  include
)

ament_package()