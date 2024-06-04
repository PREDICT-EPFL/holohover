cmake_minimum_required(VERSION 3.8)
project(holohover_drivers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set build type to RELEASE by default
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()
message(STATUS "Building ${CMAKE_BUILD_TYPE}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(holohover_msgs REQUIRED)

add_executable(control_driver src/control_driver_node.cpp src/msp.cpp)
ament_target_dependencies(control_driver rclcpp holohover_msgs)

install(TARGETS
  control_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
