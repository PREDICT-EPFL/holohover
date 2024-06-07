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

add_executable(holohover_driver src/holohover_driver_node.cpp src/msp.cpp src/pmw3389dm.cpp)
target_include_directories(holohover_driver PRIVATE kernel_modules)
ament_target_dependencies(holohover_driver rclcpp holohover_msgs)
target_link_libraries(holohover_driver libgpiod::libgpiod)

install(TARGETS
  holohover_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
