/*******************************************************************************
 * @file    config-server.h
 * @brief   A HTTP server that can be used to configure the car.
 ******************************************************************************/

#pragma once

#include <stdbool.h>

/** @defgroup config-server Configuration Server */

/**
 * @addtogroup config-server
 * @brief The configuration server is an HTTP server running on the ESP32. It is
 * used to change configuration values for the vehicle that don't require
 * reflashing.
 *
 * Ths server enables configuration of certain values by presenting a web
 * interface that contains an HTML form. The HTML form's contents are POSTed to
 * the server again, who then parses the request. The arguments are decoded and
 * then committed to the @link{nvs-manager}'s non-volatile storage.
 *
 * If the configuration is started when the ESP32 is its own access point, then
 * the configuration interface will be available at http://192.168.4.1/config.
 * @{
 */

/**
 * @brief Starts the configuration server.
 *
 * The configuration server is an HTTP server running on the IP of the ESP32. To
 * serve clients, a new client with a stack byte of 4kiB and task priority (IDLE
 * + 5) will be created.
 *
 * The configuration server just runs on the network the ESP32 is connected to.
 * If the ESP32 is acting as an access point, its IP will be the IP of the
 * router. Otherwise, the configuration server will just be available in the
 * local network (which means that the user will need to figure out the IP of
 * the vehicle to access the configuration page).
 *
 * @returns true if the configuration server started successfully, else false
 */
bool config_server_start(void);

/** @} */
