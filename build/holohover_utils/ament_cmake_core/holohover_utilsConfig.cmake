# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_holohover_utils_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED holohover_utils_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(holohover_utils_FOUND FALSE)
  elseif(NOT holohover_utils_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(holohover_utils_FOUND FALSE)
  endif()
  return()
endif()
set(_holohover_utils_CONFIG_INCLUDED TRUE)

# output package information
if(NOT holohover_utils_FIND_QUIETLY)
  message(STATUS "Found holohover_utils: 0.0.0 (${holohover_utils_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'holohover_utils' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${holohover_utils_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(holohover_utils_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${holohover_utils_DIR}/${_extra}")
endforeach()
