cmake_minimum_required(VERSION 3.8)
project(holohover_playground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(holohover_msgs REQUIRED)
find_package(holohover_common REQUIRED)
find_package(visualization_msgs REQUIRED)


find_package(piqp REQUIRED)
find_package(casadi REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

add_executable(control_mpc src/holohover_mpc_node.cpp)
#target_include_directories(control_mpc PRIVATE external/laopt/include)
ament_target_dependencies(control_mpc rclcpp geometry_msgs holohover_msgs holohover_common visualization_msgs) 
target_compile_options(control_mpc PRIVATE "-Wno-unused-parameter")
target_link_libraries(control_mpc yaml-cpp casadi)

set_target_properties(control_mpc PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "/usr/local/lib"
)
install(TARGETS
  control_mpc
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
