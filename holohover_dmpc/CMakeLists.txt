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

find_package(tf2_ros REQUIRED)
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
# set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "" FORCE)

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

# DMPC
set(QPOASES_LIBRARY_DIR "/usr/local/lib" CACHE FILEPATH "libqpOASES.so should be located in the directory \${QPOASES_LIBRARY_DIR}")
set(QPOASES_INCLUDE_DIR "/usr/local/include" CACHE FILEPATH "headers of qpOASES should be in the directory \${QPOASES_LIBRARY_DIR} and its subfolder qpOASES") 
find_library(QPOASES_LIB qpOASES ${QPOASES_LIBRARY_DIR} NO_DEFAULT_PATH)

# Get package info using pkg-config
find_package(PkgConfig)
pkg_search_module(CASADI casadi)
if(NOT CASADI_LIBRARIES)
  message(SEND_ERROR "Could not find casadi.pc - did you set PKG_CONFIG_PATH?")
endif()

message("CASADI_LIBRARY_DIRS: ${CASADI_LIBRARY_DIRS}")
message("CASADI_LIBRARIES: ${CASADI_LIBRARIES}")
message("CASADI_INCLUDEDIR: ${CASADI_INCLUDEDIR}")
link_directories(${CASADI_LIBRARY_DIRS})
include_directories("${QPOASES_INCLUDE_DIR}")

add_executable(control_dmpc src/holohover_dmpc_admm_node.cpp src/sProb.cpp src/doptTimer.cpp)
target_include_directories(control_dmpc PRIVATE ${CASADI_INCLUDEDIR})
ament_target_dependencies(control_dmpc rclcpp geometry_msgs holohover_msgs holohover_common)
target_link_libraries(control_dmpc ${QPOASES_LIB} casadi yaml-cpp)

add_executable(dmpc_trigger src/holohover_dmpc_trigger_node.cpp)
ament_target_dependencies(dmpc_trigger rclcpp std_msgs)
target_link_libraries(dmpc_trigger yaml-cpp)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  control_dmpc dmpc_trigger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
