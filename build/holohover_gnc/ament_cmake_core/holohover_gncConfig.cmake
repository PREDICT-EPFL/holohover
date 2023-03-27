# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_holohover_gnc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED holohover_gnc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(holohover_gnc_FOUND FALSE)
  elseif(NOT holohover_gnc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(holohover_gnc_FOUND FALSE)
  endif()
  return()
endif()
set(_holohover_gnc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT holohover_gnc_FIND_QUIETLY)
  message(STATUS "Found holohover_gnc: 0.0.0 (${holohover_gnc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'holohover_gnc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${holohover_gnc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(holohover_gnc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${holohover_gnc_DIR}/${_extra}")
endforeach()
