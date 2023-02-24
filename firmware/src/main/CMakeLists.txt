idf_component_register(
    SRCS main.c msp.c PMW3389DM/spi_pmw3389dm.c
    INCLUDE_DIRS ""
    REQUIRES micro_ros_espidf_component nvs-manager wifi config-server
)