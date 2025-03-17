cmake_minimum_required(VERSION 3.8)
project(holohover_dmpc)

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
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(piqp REQUIRED)
find_package(casadi REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(geometry_msgs)
find_package(holohover_msgs REQUIRED)
find_package(holohover_common REQUIRED)

find_package(quill REQUIRED)

#set build type to RELEASE by default
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

add_library(control_dmpc_lib src/sparse_casadi_function_buffer.cpp src/vector_casadi_function_buffer.cpp src/sProb.cpp
                             src/doptTimer.cpp src/rcl_timer_helper.c)
ament_target_dependencies(control_dmpc_lib rclcpp)
target_link_libraries(control_dmpc_lib casadi yaml-cpp Eigen3::Eigen)

add_executable(control_dmpc_admm src/holohover_dmpc_admm_node.cpp)
ament_target_dependencies(control_dmpc_admm rclcpp geometry_msgs holohover_msgs holohover_common)
target_link_libraries(control_dmpc_admm control_dmpc_lib quill::quill piqp::piqp)

add_executable(control_dmpc_dsqp src/holohover_dmpc_dsqp_node.cpp)
ament_target_dependencies(control_dmpc_dsqp rclcpp geometry_msgs holohover_msgs holohover_common)
target_link_libraries(control_dmpc_dsqp control_dmpc_lib quill::quill piqp::piqp)

add_executable(dmpc_trigger src/holohover_dmpc_trigger_node.cpp)
ament_target_dependencies(dmpc_trigger rclcpp std_msgs)
target_link_libraries(dmpc_trigger yaml-cpp)

add_executable(trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp holohover_msgs)
target_link_libraries(trajectory_generator yaml-cpp)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ocp_specs
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  control_dmpc_admm dmpc_trigger trajectory_generator control_dmpc_dsqp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
