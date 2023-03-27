# Install script for directory: /home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/acl3-ubuntu/holohover_ws/src/holohover/install/soem")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/acl3-ubuntu/holohover_ws/src/holohover/build/soem/SOEM/libsoem.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/soem" TYPE FILE FILES
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercat.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatbase.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatcoe.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatconfig.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatconfiglist.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatdc.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercateoe.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatfoe.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatmain.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatprint.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercatsoe.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/soem/ethercattype.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/osal/linux/osal_defs.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/osal/osal.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/oshw/linux/nicdrv.h"
    "/home/acl3-ubuntu/holohover_ws/src/holohover/soem/SOEM/oshw/linux/oshw.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/acl3-ubuntu/holohover_ws/src/holohover/build/soem/SOEM/test/linux/slaveinfo/cmake_install.cmake")
  include("/home/acl3-ubuntu/holohover_ws/src/holohover/build/soem/SOEM/test/linux/eepromtool/cmake_install.cmake")
  include("/home/acl3-ubuntu/holohover_ws/src/holohover/build/soem/SOEM/test/linux/simple_test/cmake_install.cmake")

endif()

