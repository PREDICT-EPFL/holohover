#pragma once

#include <stdio.h>

#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>

bool bluetooth_serial_init(const char *deviceName);
bool bluetooth_serial_available();
int bluetooth_serial_peak();
bool bluetooth_serial_has_client(void);
int bluetooth_serial_read();
size_t bluetooth_serial_write(const uint8_t *buffer, size_t size);
void bluetooth_serial_flush();
void bluetooth_serial_end();
void bluetooth_serial_confirmReply(bool confirm);
esp_err_t bluetooth_serial_register_callback(esp_spp_cb_t * callback);
void bluetooth_serial_enableSSP();
bool bluetooth_serial_setPin(const char *pin);
bool bluetooth_serial_connect_name(const char * remoteName);
bool bluetooth_serial_connect_address(uint8_t remoteAddress[]);
bool bluetooth_serial_connect();
bool bluetooth_serial_disconnect();
bool bluetooth_serial_unpair_device(uint8_t remoteAddress[]);
bool bluetooth_serial_connected(int timeout);
bool bluetooth_serial_is_ready(bool checkMaster, int timeout);
