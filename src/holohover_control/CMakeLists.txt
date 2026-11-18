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
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
# TODO: [Lyrical] ament_target_dependencies() was removed in Lyrical, replaced by target_link_libraries()
# with the dependencies' CMake targets (pkg::pkg, ${msg_pkg_TARGETS})
target_link_libraries(control_lqr rclcpp::rclcpp ${geometry_msgs_TARGETS} ${holohover_msgs_TARGETS} holohover_common::holohover_common)
target_link_libraries(control_lqr yaml-cpp)

# add_executable(control_signal src/control_signal_node.cpp)
# target_link_libraries(control_signal rclcpp::rclcpp ${geometry_msgs_TARGETS} ${holohover_msgs_TARGETS} holohover_common::holohover_common)
# target_link_libraries(control_signal yaml-cpp)

add_executable(control_exp src/control_exp_node.cpp)
target_link_libraries(control_exp rclcpp::rclcpp ${geometry_msgs_TARGETS} ${holohover_msgs_TARGETS} holohover_common::holohover_common)
target_link_libraries(control_exp yaml-cpp)

add_executable(control_teleop src/control_teleop_node.cpp)
target_link_libraries(control_teleop rclcpp::rclcpp ament_index_cpp::ament_index_cpp ${holohover_msgs_TARGETS} holohover_common::holohover_common)
target_link_libraries(control_teleop yaml-cpp)

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS
  control_lqr control_exp control_teleop # control_signal
  DESTINATION lib/${PROJECT_NAME}
)

# Python controllers (holohover_control/) and their nodes
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/control_plan scripts/teleop DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_holohover_control test)  # test_parity.py also needs holohover_planner
  else()
    message(STATUS "ament_cmake_pytest not found: run test/ with python3 -m pytest")
  endif()
endif()

ament_package()
