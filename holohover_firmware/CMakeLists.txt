cmake_minimum_required(VERSION 3.5)

set (EXTRA_COMPONENT_DIRS
    src/config-server
    src/main
    src/micro_ros_espidf_component
    src/msp
    src/neopixel
    src/nvs-manager
    src/pmw3389dm
    src/wifi
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(holohover_firmware)
