cmake_minimum_required(VERSION 3.8)
project(holohover_navigation)

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
find_package(yaml-cpp REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(geometry_msgs)
find_package(holohover_msgs REQUIRED)
find_package(holohover_common REQUIRED)

#set build type to RELEASE by default
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

#set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

add_executable(navigation src/navigation_node.cpp)
ament_target_dependencies(navigation rclcpp tf2_ros geometry_msgs holohover_msgs holohover_common)
target_link_libraries(navigation yaml-cpp)

add_executable(navigation_disturbance src/navigation_disturbance_node.cpp)
ament_target_dependencies(navigation_disturbance rclcpp tf2_ros geometry_msgs holohover_msgs holohover_common)
target_link_libraries(navigation_disturbance yaml-cpp)

#install(DIRECTORY
#  config
#  DESTINATION share/${PROJECT_NAME}
#)

install(TARGETS
  navigation navigation_disturbance
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
