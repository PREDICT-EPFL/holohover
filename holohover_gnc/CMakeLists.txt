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

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(geometry_msgs)
find_package(ros_polympc REQUIRED)
find_package(holohover_msgs REQUIRED)

include_directories(include)

add_executable(simulation src/simulation/simulation_node.cpp)
ament_target_dependencies(simulation rclcpp tf2_ros geometry_msgs holohover_msgs ros_polympc)
target_link_libraries(simulation Eigen3::Eigen)

add_executable(navigation src/navigation/navigation_node.cpp)
ament_target_dependencies(navigation rclcpp tf2_ros geometry_msgs holohover_msgs ros_polympc)
target_link_libraries(navigation Eigen3::Eigen)

add_executable(control_lqr src/control/control_lqr_node.cpp)
ament_target_dependencies(control_lqr rclcpp geometry_msgs holohover_msgs ros_polympc)
target_link_libraries(control_lqr Eigen3::Eigen)

add_executable(control_signal src/control/control_signal_node.cpp)
ament_target_dependencies(control_signal rclcpp geometry_msgs holohover_msgs ros_polympc)
target_link_libraries(control_signal Eigen3::Eigen)

add_executable(control_exp src/control/control_exp_node.cpp)
ament_target_dependencies(control_exp rclcpp geometry_msgs holohover_msgs ros_polympc)
target_link_libraries(control_exp Eigen3::Eigen)

install(DIRECTORY include/
        DESTINATION include
        )
ament_export_include_directories(include)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        simulation navigation control_lqr control_signal control_exp
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
