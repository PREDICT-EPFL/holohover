cmake_minimum_required(VERSION 3.8)
project(holohover_common)

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
find_package(Eigen3 REQUIRED)
find_package(piqp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

# Find pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v2.12.0.zip
)
FetchContent_MakeAvailable(pybind11)

#set build type to RELEASE by default
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

add_library(holohover_common INTERFACE)
target_link_libraries(holohover_common INTERFACE Eigen3::Eigen piqp::piqp)

pybind11_add_module(bindings src/python_bindings.cpp)
target_include_directories(bindings PRIVATE include)
target_link_libraries(bindings PRIVATE Eigen3::Eigen piqp::piqp yaml-cpp)

ament_export_include_directories(include)
ament_export_targets(holohover_commonTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 piqp)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS holohover_common
  EXPORT holohover_commonTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS bindings
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
