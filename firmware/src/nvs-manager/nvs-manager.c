/*******************************************************************************
 * @file    nvs-manager.c
 * @brief   Initializes and manages the non-volatile storage (NVS).
 ******************************************************************************/

#include "nvs-manager.h"

#include <string.h>

#include "nvs_flash.h"
#include "sdkconfig.h"

/* Global variable declaration ---------------------------------------------- */

struct Configuration global_config = {0};

/* Local variables ---------------------------------------------------------- */

/** Handle to the storage where the vehicle config is stored. */
nvs_handle_t vehicle_config_handle = 0;

/* Public function implementation ------------------------------------------- */

void nvs_init(void) {

    // Call ESP-IDF initialization. If it fails, need to erase flash and
    // re-init.
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased. If it fails,
        // abort() the boot, because it is a more serious issue.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // Check again if it worked. If it didn't, abort.
    ESP_ERROR_CHECK(err);

    // Open the container used to store the configuration
    err = nvs_open("vehicle-storage", NVS_READWRITE, &vehicle_config_handle);
    ESP_ERROR_CHECK(err);

    return;
}

void nvs_load_config(void) {
    size_t agent_ip_len = sizeof(global_config.agent_ip);
    esp_err_t ret = nvs_get_str(vehicle_config_handle, "agent_ip",
                                global_config.agent_ip, &agent_ip_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.agent_ip, CONFIG_MICRO_ROS_AGENT_IP,
                sizeof(global_config.agent_ip));
        // Zero-terminate in case the value in the config would be an overflow.
        // What will happen is that it just doesn't work, but instead can still
        // be set via the web interface.
        global_config.agent_ip[sizeof(global_config.agent_ip) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    ret = nvs_get_u16(vehicle_config_handle, "agent_port", &global_config.agent_port);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.agent_port = atoi(CONFIG_MICRO_ROS_AGENT_PORT);
    }

    size_t ssid_len = sizeof(global_config.ssid);
    ret = nvs_get_str(vehicle_config_handle, "ssid", global_config.ssid,
                      &ssid_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.ssid, CONFIG_ESP_WIFI_SSID,
                sizeof(global_config.ssid));
        global_config.ssid[sizeof(global_config.ssid) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    size_t pwd_len = sizeof(global_config.pwd);
    ret = nvs_get_str(vehicle_config_handle, "pwd", global_config.pwd, &pwd_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.pwd, CONFIG_ESP_WIFI_PASSWORD,
                sizeof(global_config.pwd));
        global_config.pwd[sizeof(global_config.pwd) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    size_t static_ip_len = sizeof(global_config.static_ip);
    ret = nvs_get_str(vehicle_config_handle, "static_ip", global_config.static_ip, &static_ip_len);

    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            strncpy(global_config.static_ip, "0.0.0.0", sizeof(global_config.static_ip));
            // Zero-terminate in case the value in the config would be an overflow.
            // What will happen is that it just doesn't work, but instead can still
            // be set via the web interface.
            global_config.static_ip[sizeof(global_config.static_ip) - 1] = '\0';
            break;
        default:
            ESP_ERROR_CHECK(ret);
    }

    size_t static_netmask_len = sizeof(global_config.static_netmask);
    ret = nvs_get_str(vehicle_config_handle, "static_netmask", global_config.static_netmask, &static_netmask_len);

    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            strncpy(global_config.static_netmask, "255.255.255.0", sizeof(global_config.static_netmask));
            // Zero-terminate in case the value in the config would be an overflow.
            // What will happen is that it just doesn't work, but instead can still
            // be set via the web interface.
            global_config.static_netmask[sizeof(global_config.static_netmask) - 1] = '\0';
            break;
        default:
            ESP_ERROR_CHECK(ret);
    }

    size_t static_gw_len = sizeof(global_config.static_gw);
    ret = nvs_get_str(vehicle_config_handle, "static_gw", global_config.static_gw, &static_gw_len);

    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            strncpy(global_config.static_gw, "192.168.0.1", sizeof(global_config.static_gw));
            // Zero-terminate in case the value in the config would be an overflow.
            // What will happen is that it just doesn't work, but instead can still
            // be set via the web interface.
            global_config.static_gw[sizeof(global_config.static_gw) - 1] = '\0';
            break;
        default:
            ESP_ERROR_CHECK(ret);
    }
}

void nvs_store_config() {
    ESP_ERROR_CHECK(nvs_set_str(vehicle_config_handle, "agent_ip", global_config.agent_ip));
    ESP_ERROR_CHECK(nvs_set_u16(vehicle_config_handle, "agent_port", global_config.agent_port));
    ESP_ERROR_CHECK(nvs_set_str(vehicle_config_handle, "ssid", global_config.ssid));
    ESP_ERROR_CHECK(nvs_set_str(vehicle_config_handle, "pwd", global_config.pwd));
    ESP_ERROR_CHECK(nvs_set_str(vehicle_config_handle, "static_ip", global_config.static_ip));
    ESP_ERROR_CHECK(nvs_set_str(vehicle_config_handle, "static_netmask", global_config.static_netmask));
    ESP_ERROR_CHECK(nvs_set_str(vehicle_config_handle, "static_gw", global_config.static_gw));

    ESP_ERROR_CHECK(nvs_commit(vehicle_config_handle));
}

/* Compile-time asserts ----------------------------------------------------- */

// This asserts are needed because floats are stored as uint32_t's in NVS as
// they're assumed to be the same size. If this assumption is broken, stop
// compilation.
_Static_assert(sizeof(float) == sizeof(uint32_t), "float is not four bytes");
