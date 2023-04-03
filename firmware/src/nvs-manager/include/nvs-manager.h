/*******************************************************************************
 * @file    nvs-manager.h
 * @brief   Initializes and manages the non-volatile storage (NVS).
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <sys/types.h>

/** @defgroup nvs-manager Non-volatile Storage Manager */

/**
 * @addtogroup nvs-manager
 * @brief The NVS Manager manages the storage of non-volatile variables.
 *
 * Initialize the NVS by calling nvs_init(). If the function returns,
 * setup was successful.
 * @{
 */

/**
 * @brief Set of configuration values that are stored non-volatile.
 *
 * When the device's non-volatile storage is initialized, the set of
 * configuration values is loaded from storage. They can be changed during
 * runtime and committed back to NVS.
 */
struct Configuration {
    char agent_ip[40];              ///< Agent IP address in human-readable format.
    uint16_t agent_port;            ///< UDP port of agent computer
    char ssid[33];                  ///< SSID
    char pwd[100];                  ///< Password
    char static_ip[40];             ///< Static IP address in human-readable format. For dynamic IP set to 0.0.0.0
    char static_netmask[40];        ///< Static netmask address in human-readable format.
    char static_gw[40];             ///< Static gateway address in human-readable format.
};

/**
 * @brief Global configuration struct.
 */
extern struct Configuration global_config;

/* Public function declaration ---------------------------------------------- */

/**
 * @brief Initializes the non-volatile storage unit of the ESP32.
 *
 * This calls the ESP-IDF setup functionality of the nvs. If this fails, boot
 * is aborted, since it is very likely that a component of the ESP32 is damaged
 * or there is a more serious issue with the flash memory (overwritten by
 * binary?).
 */
void nvs_init(void);

/**
 * @brief Loads the current configuration from NVS.
 */
void nvs_load_config(void);

/**
 * @brief Commit the changes of the current configuration to NVS.
 */
void nvs_store_config(void);

/** @} */
