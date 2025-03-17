cmake_minimum_required(VERSION 3.8)
project(holohover_drivers)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
find_package(libgpiod REQUIRED)

add_executable(holohover_fc src/holohover_fc_node.cpp src/msp.cpp)
ament_target_dependencies(holohover_fc rclcpp holohover_msgs)

add_executable(holohover_mouse_sensor src/holohover_mouse_sensor_node.cpp src/pmw3389dm.cpp)
target_include_directories(holohover_mouse_sensor PRIVATE kernel_modules)
ament_target_dependencies(holohover_mouse_sensor rclcpp holohover_msgs)
target_link_libraries(holohover_mouse_sensor libgpiod::libgpiod)

install(TARGETS
        holohover_fc holohover_mouse_sensor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
