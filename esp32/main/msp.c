/*
  msp.cpp

  Copyright (c) 2017, Fabrizio Di Vittorio (fdivitto2013@gmail.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "msp.h"
#include <time.h>

void msp_write(uart_port_t uart_num, uint8_t message_id, void* payload, size_t size)
{
    uart_write_bytes(uart_num, "$M<", 3);
    uart_write_bytes(uart_num, &size, 1);
    uart_write_bytes(uart_num, &message_id, 1);
    uint8_t checksum = size ^ message_id;
    uint8_t * payload_ptr = (uint8_t*) payload;
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t b = *(payload_ptr++);
        checksum ^= b;
    }
    uart_write_bytes(uart_num, payload, size);
    uart_write_bytes(uart_num, &checksum, 1);
}


int msp_read(uart_port_t uart_num, uint8_t* message_id, void* payload, uint32_t payload_size, TickType_t ticks_to_wait)
{
    TickType_t ticks_end = xTaskGetTickCount() + ticks_to_wait;
    TickType_t ticks_remaining;
    uint8_t message_size;
    while (1) {

        // read header
        size_t length;
        do {
            ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, &length));
            ticks_remaining = ticks_end - xTaskGetTickCount();
            if (ticks_remaining > ticks_to_wait) // ticks_to_wait will underflow once xTaskGetTickCount() > ticks_end
                return -1;
        } while (length < 6);

        uint8_t header[3];
        if (uart_read_bytes(uart_num, header, 3, ticks_remaining) < 0)
            return -1;

        // check header
        if (header[0] == '$' && header[1] == 'M' && header[2] == '>') {
            // header ok, read payload size
            if (uart_read_bytes(uart_num, &message_size, 1, ticks_remaining) < 0)
                return -1;

            // read message ID (type)
            if (uart_read_bytes(uart_num, &message_id, 1, ticks_remaining) < 0)
                return -1;

            uint8_t checksum_calc = message_size ^ *message_id;

            // read payload
            ticks_remaining = ticks_end - xTaskGetTickCount();
            if (uart_read_bytes(uart_num, payload, payload_size, ticks_remaining) < 0)
                return -1;

            uint8_t * payload_ptr = (uint8_t*) payload;
            uint8_t idx = 0;
            while (idx < message_size) {
                uint8_t b;
                if (idx < payload_size) {
                    b = *(payload_ptr++);
                } else {
                    ticks_remaining = ticks_end - xTaskGetTickCount();
                    if (uart_read_bytes(uart_num, &b, 1, ticks_remaining) < 0)
                        return -1;
                }
                checksum_calc ^= b;
                ++idx;
            }
            // zero remaining bytes if *size < maxSize
            for (; idx < payload_size; ++idx)
                *(payload_ptr++) = 0;

            // read and check checksum
            uint8_t checksum;
            ticks_remaining = ticks_end - xTaskGetTickCount();
            if (uart_read_bytes(uart_num, &checksum, 1, ticks_remaining) < 0)
                return -1;

            if (checksum_calc == checksum) {
                return message_size;
            }
        }
    }
}

int msp_wait(uart_port_t uart_num, uint8_t message_id, void * payload, uint32_t payload_size, TickType_t ticks_to_wait)
{
    TickType_t ticks_end = xTaskGetTickCount() + ticks_to_wait;
    TickType_t ticks_remaining = ticks_to_wait;

    uint8_t read_message_id;
    int message_size;
    while (ticks_remaining <= ticks_to_wait) {
        message_size = msp_read(uart_num, &read_message_id, payload, payload_size, ticks_remaining);
        if (message_size >= 0 && message_id == read_message_id)
            return message_size;

        ticks_remaining = ticks_end - xTaskGetTickCount();
    }

    // timeout
    return -1;
}


// send a message and wait for the reply
int msp_request(uart_port_t uart_num, uint8_t message_id, void* payload, uint32_t payload_size, TickType_t ticks_to_wait)
{
    msp_write(uart_num, message_id, NULL, 0);
    return msp_wait(uart_num, message_id, payload, payload_size, ticks_to_wait);
}


// send message and wait for ack
int msp_command(uart_port_t uart_num, uint8_t message_id, void* payload, uint32_t payload_size, TickType_t ticks_to_wait)
{
    TickType_t ticks_end = xTaskGetTickCount() + ticks_to_wait;
    msp_write(uart_num, message_id, payload, payload_size);
    TickType_t ticks_remaining = ticks_end - xTaskGetTickCount();

    // ack required
    if (ticks_to_wait > 0)
        return msp_wait(uart_num, message_id, NULL, 0, ticks_remaining);

    return -1;
}


// map MSP_MODE_xxx to box ids
// mixed values from cleanflight and inav
static const uint8_t BOXIDS[30] = {
        0,  //  0: MSP_MODE_ARM
        1,  //  1: MSP_MODE_ANGLE
        2,  //  2: MSP_MODE_HORIZON
        3,  //  3: MSP_MODE_NAVALTHOLD (cleanflight BARO)
        5,  //  4: MSP_MODE_MAG
        6,  //  5: MSP_MODE_HEADFREE
        7,  //  6: MSP_MODE_HEADADJ
        8,  //  7: MSP_MODE_CAMSTAB
        10, //  8: MSP_MODE_NAVRTH (cleanflight GPSHOME)
        11, //  9: MSP_MODE_NAVPOSHOLD (cleanflight GPSHOLD)
        12, // 10: MSP_MODE_PASSTHRU
        13, // 11: MSP_MODE_BEEPERON
        15, // 12: MSP_MODE_LEDLOW
        16, // 13: MSP_MODE_LLIGHTS
        19, // 14: MSP_MODE_OSD
        20, // 15: MSP_MODE_TELEMETRY
        21, // 16: MSP_MODE_GTUNE
        22, // 17: MSP_MODE_SONAR
        26, // 18: MSP_MODE_BLACKBOX
        27, // 19: MSP_MODE_FAILSAFE
        28, // 20: MSP_MODE_NAVWP (cleanflight AIRMODE)
        29, // 21: MSP_MODE_AIRMODE (cleanflight DISABLE3DSWITCH)
        30, // 22: MSP_MODE_HOMERESET (cleanflight FPVANGLEMIX)
        31, // 23: MSP_MODE_GCSNAV (cleanflight BLACKBOXERASE)
        32, // 24: MSP_MODE_HEADINGLOCK
        33, // 25: MSP_MODE_SURFACE
        34, // 26: MSP_MODE_FLAPERON
        35, // 27: MSP_MODE_TURNASSIST
        36, // 28: MSP_MODE_NAVLAUNCH
        37, // 29: MSP_MODE_AUTOTRIM
};


// returns active mode (using MSP_STATUS and MSP_BOXIDS messages)
// see MSP_MODE_... for bits inside activeModes
int msp_get_active_modes(uart_port_t uart_num, uint32_t* activeModes, TickType_t ticks_to_wait)
{
    TickType_t ticks_end = xTaskGetTickCount() + ticks_to_wait;
    // request status ex
    struct msp_status_t status;
    if (msp_request(uart_num, MSP_STATUS, &status, sizeof(status), ticks_to_wait) >= 0) {
        // request permanent ids associated to boxes
        uint8_t ids[sizeof(BOXIDS)];
        TickType_t ticks_remaining = ticks_end - xTaskGetTickCount();
        int message_size;
        message_size = msp_request(uart_num, MSP_BOXIDS, ids, sizeof(ids), ticks_remaining);
        if (message_size >= 0) {
            // compose activeModes, converting BOXIDS to bit map (setting 1 if related flag in flightModeFlags is set)
            *activeModes = 0;
            for (uint8_t i = 0; i < message_size; ++i) {
                if (status.flightModeFlags & (1 << i)) {
                    for (uint8_t j = 0; j < sizeof(BOXIDS); ++j) {
                        if (BOXIDS[j] == ids[i]) {
                            *activeModes |= 1 << j;
                            break;
                        }
                    }
                }
            }
            return 0;
        }
    }

    return -1;
}
