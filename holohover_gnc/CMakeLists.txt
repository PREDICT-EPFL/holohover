cmake_minimum_required(VERSION 3.8)
project(holohover_gnc)

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
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(submodules/polympc/src)
include_directories(include)

add_executable(holohover_navigation src/navigation/holohover_navigation_node.cpp)
target_link_libraries(holohover_navigation Eigen3::Eigen)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
