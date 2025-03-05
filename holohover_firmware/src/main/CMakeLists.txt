idf_component_register(
    SRCS main.c
    INCLUDE_DIRS ""
    REQUIRES config-server micro_ros_espidf_component msp neopixel nvs-manager pmw3389dm wifi
)