idf_component_register(
        SRCS main.c msp.c PMW3389DM/spi_pmw3389dm.c bluetooth/bluetooth_serial.c bluetooth/esp32_bluetooth_serial_transport.c esp32_serial_transport.c
        INCLUDE_DIRS ""
)
