# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rokubimini_ethercat_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rokubimini_ethercat_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rokubimini_ethercat_FOUND FALSE)
  elseif(NOT rokubimini_ethercat_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rokubimini_ethercat_FOUND FALSE)
  endif()
  return()
endif()
set(_rokubimini_ethercat_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rokubimini_ethercat_FIND_QUIETLY)
  message(STATUS "Found rokubimini_ethercat: 0.6.1 (${rokubimini_ethercat_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rokubimini_ethercat' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rokubimini_ethercat_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rokubimini_ethercat_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${rokubimini_ethercat_DIR}/${_extra}")
endforeach()
