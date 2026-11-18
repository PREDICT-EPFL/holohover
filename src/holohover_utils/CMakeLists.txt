cmake_minimum_required(VERSION 3.8)
project(holohover_utils)

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
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(holohover_msgs REQUIRED)
find_package(holohover_common REQUIRED)

add_executable(rviz_interface src/rviz_interface_node.cpp)
# TODO: [Lyrical] ament_target_dependencies() was removed in Lyrical, replaced by target_link_libraries()
# with the dependencies' CMake targets (pkg::pkg, ${msg_pkg_TARGETS})
target_link_libraries(rviz_interface rclcpp::rclcpp tf2::tf2 ${visualization_msgs_TARGETS} ${holohover_msgs_TARGETS} holohover_common::holohover_common ${geometry_msgs_TARGETS})
target_link_libraries(rviz_interface yaml-cpp)

add_executable(optitrack_interface src/optitrack_interface_node.cpp)
target_link_libraries(optitrack_interface rclcpp::rclcpp ${geometry_msgs_TARGETS} tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs holohover_common::holohover_common)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config gui launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  rviz_interface optitrack_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  holohover_utils/ping_estimator.py
  holohover_utils/air_hockey_evaluate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
