# - Find libgpiod
# Find the native libgpiod headers and libraries.
#
#  LIBGPIOD_DIR - set this to where to look for libgpiod
#
#  LIBGPIOD_INCLUDE_DIRS - where to find gpiod.h, etc.
#  LIBGPIOD_LIBRARIES    - List of libraries when using libgpiod.
#  libgpiod_FOUND        - True if libgpiod found.

# Look for the header file.
find_path(LIBGPIOD_INCLUDE_DIR
  HINTS ${LIBGPIOD_DIR}/include
  NAMES gpiod.h)
mark_as_advanced(LIBGPIOD_INCLUDE_DIR)

# Look for the library.
find_library(LIBGPIOD_LIBRARY
  HINTS ${LIBGPIOD_DIR}/lib ${LIBGPIOD_DIR}/lib64
  NAMES gpiod)
mark_as_advanced(LIBGPIOD_LIBRARY)

# handle the QUIETLY and REQUIRED arguments and set LIBGPIOD_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libgpiod DEFAULT_MSG LIBGPIOD_LIBRARY LIBGPIOD_INCLUDE_DIR)

if(libgpiod_FOUND)
  set(LIBGPIOD_LIBRARIES ${LIBGPIOD_LIBRARY})
  set(LIBGPIOD_INCLUDE_DIRS ${LIBGPIOD_INCLUDE_DIR})
  if(NOT TARGET libgpiod::libgpiod)
    add_library(libgpiod::libgpiod UNKNOWN IMPORTED)
    set_target_properties(libgpiod::libgpiod PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBGPIOD_INCLUDE_DIR}")
    set_property(TARGET libgpiod::libgpiod APPEND PROPERTY IMPORTED_LOCATION "${LIBGPIOD_LIBRARY}")
  endif()
else()
  set(LIBGPIOD_LIBRARIES)
  set(LIBGPIOD_INCLUDE_DIRS)
endif()