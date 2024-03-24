cmake_minimum_required(VERSION 3.8)
project(holohover_control)

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
find_package(Eigen3 REQUIRED)

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

add_executable(control_lqr src/control_lqr_node.cpp)
ament_target_dependencies(control_lqr rclcpp geometry_msgs holohover_msgs holohover_common)
target_link_libraries(control_lqr yaml-cpp)

add_executable(control_signal src/control_signal_node.cpp)
ament_target_dependencies(control_signal rclcpp geometry_msgs holohover_msgs holohover_common)
target_link_libraries(control_signal yaml-cpp)

add_executable(control_exp src/control_exp_node.cpp)
ament_target_dependencies(control_exp rclcpp geometry_msgs holohover_msgs holohover_common)
target_link_libraries(control_exp yaml-cpp)

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS
  control_lqr control_signal control_exp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
