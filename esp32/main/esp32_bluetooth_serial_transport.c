#include <uxr/client/transport.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bluetooth_serial.h"

#define TAG "HOLOHOVER_ESP32_BLUETOOTH_SERIAL_TRANSPORT"

bool esp32_bluetooth_serial_open(struct uxrCustomTransport * transport){
    const char * device_name = (const char*) transport->args;
    if (!bluetooth_serial_init(device_name)) {
        return false;
    }
    while (!bluetooth_serial_has_client()) {
        ESP_LOGI(TAG, "Trying to connect to client...");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    return true;
}

bool esp32_bluetooth_serial_close(struct uxrCustomTransport * transport){
    bluetooth_serial_end();
    return true;
}

size_t esp32_bluetooth_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    return bluetooth_serial_write(buf, len);
}

size_t esp32_bluetooth_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    TickType_t ticks_to_wait = timeout / portTICK_RATE_MS;
    TickType_t ticks_end = xTaskGetTickCount() + ticks_to_wait;
    TickType_t ticks_remaining = ticks_to_wait;

    int idx = 0;
    while (idx < len && ticks_remaining <= ticks_to_wait) {
        int c = bluetooth_serial_read();
        if (c >= 0) {
            *(buf++) = c;
        }
        idx++;
        ticks_remaining = ticks_end - xTaskGetTickCount();
    }
    return idx;
}
