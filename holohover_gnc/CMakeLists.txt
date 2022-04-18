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

find_package(holohover_msgs REQUIRED)

include_directories(submodules/polympc/src)
include_directories(include)

add_executable(navigation src/navigation/navigation_node.cpp)
ament_target_dependencies(navigation rclcpp holohover_msgs)
target_link_libraries(navigation Eigen3::Eigen)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        navigation
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
