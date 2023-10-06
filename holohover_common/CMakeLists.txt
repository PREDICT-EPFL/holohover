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

#set build type to RELEASE by default
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

add_library(holohover_common INTERFACE)
target_link_libraries(holohover_common INTERFACE Eigen3::Eigen piqp::piqp)

ament_export_include_directories(include)
ament_export_targets(holohover_commonTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 piqp)

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

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
