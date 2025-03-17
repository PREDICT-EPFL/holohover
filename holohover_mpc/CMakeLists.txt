cmake_minimum_required(VERSION 3.8)
project(holohover_mpc)

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

add_executable(control_mpc src/holohover_mpc_node.cpp)
target_include_directories(control_mpc PRIVATE external/laopt/include)
ament_target_dependencies(control_mpc rclcpp geometry_msgs holohover_msgs holohover_common)
target_compile_options(control_mpc PRIVATE "-Wno-unused-parameter")
target_link_libraries(control_mpc yaml-cpp)

add_executable(double_integrator_test src/double_integrator_test.cpp)
target_include_directories(double_integrator_test PRIVATE external/laopt/include)
ament_target_dependencies(double_integrator_test rclcpp holohover_common)
target_compile_options(double_integrator_test PRIVATE "-Wno-unused-parameter")
target_link_libraries(double_integrator_test yaml-cpp)

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS
  double_integrator_test control_mpc
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
