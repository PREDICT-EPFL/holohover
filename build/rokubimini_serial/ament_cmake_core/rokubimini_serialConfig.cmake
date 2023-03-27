# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rokubimini_serial_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rokubimini_serial_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rokubimini_serial_FOUND FALSE)
  elseif(NOT rokubimini_serial_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rokubimini_serial_FOUND FALSE)
  endif()
  return()
endif()
set(_rokubimini_serial_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rokubimini_serial_FIND_QUIETLY)
  message(STATUS "Found rokubimini_serial: 0.6.1 (${rokubimini_serial_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rokubimini_serial' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rokubimini_serial_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rokubimini_serial_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${rokubimini_serial_DIR}/${_extra}")
endforeach()
