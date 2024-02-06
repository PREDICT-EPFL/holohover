/*******************************************************************************
 * @file    config-server.c
 * @brief   A HTTP server that can be used to configure the car.
 ******************************************************************************/

#include "config-server.h"

#include <ctype.h>
#include <stdlib.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs-manager.h"

/* Externally defined git version strings ----------------------------------- */

extern const char *GIT_TAG;

/* Private function declaration --------------------------------------------- */

/**
 * @brief Handler for HTTP GET requests.
 * @param req The HTTP request to serve
 * @returns esp_err_t ESP_OK if the request could be processed, error otherwise.
 */
static esp_err_t config_server_get_handler(httpd_req_t *req);

/**
 * @brief Handler for serving the style.css.
 * @param req The HTTP request to serve
 * @returns esp_err_t ESP_OK if the request could be processed, error otherwise.
 */
static esp_err_t config_server_get_style_handler(httpd_req_t *req);

/**
 * @brief Handler for HTTP POST requests.
 * @param req The HTTP request to serve
 * @return esp_err_t ESP_OK if the request could be processed, error otherwise.
 */
static esp_err_t config_server_post_handler(httpd_req_t *req);

/**
 * @brief Handler for serving a raw text file containing the current
 * configuration values.
 *
 * The current configuration values are filled into the form to make it more
 * obvious how the car is currently configured.
 * @param req The HTTP request to serve.
 * @return esp_err_t ESP_Ok if the request could be processed, error otherwise
 */
static esp_err_t config_server_get_config_handler(httpd_req_t *req);

/**
 * @brief Decodes a percentage-encoded string into 'normal' C
 * @param dst The destination where the string's decoded contents should be
 * written
 * @param src The source string, nul-terminated
 * @copyright Source: https://stackoverflow.com/a/14530993
 */
void url_decode(char *dst, const char *src);

/**
 * @brief Updates the global configuration with a new value.
 * @param key Pointer to a string that contains the key for the config value to
 * be updated.
 * @param value The new value to for the attribute name in @p key.
 * @returns true if the config was updated and needs committing, false
 * otherwise. If no update is necessary because @p value is empty, returns
 * false.
 */
static bool config_server_update_config(const char *key, const char *value);

/* Local variables ---------------------------------------------------------- */

// The HTML file that is served to any client connecting on / is embedded
// into the flash memory on compile time and can be extracted.
// The following two pointers point to the beginning and end of the flash
// section where the file is stored.

/** Pointer to start location of embedded html file. */
extern const uint8_t configserver_start[] asm("_binary_configserver_html_start");
/** Pointer to end location of embedded html file. */
extern const uint8_t configserver_end[] asm("_binary_configserver_html_end");

/** Pointer to start location of embedded css file. */
extern const uint8_t style_start[] asm("_binary_style_css_start");
/** Pointer to end location of embedded css file. */
extern const uint8_t style_end[] asm("_binary_style_css_end");

/** URI handler for all GET requests for / */
static httpd_uri_t uri_get = {.uri = "/",
                              .method = HTTP_GET,
                              .handler = &config_server_get_handler,
                              .user_ctx = NULL};

/** URI handler for all GET requests for /style.css */
static httpd_uri_t uri_style_get = {.uri = "/style.css",
                                    .method = HTTP_GET,
                                    .handler = &config_server_get_style_handler,
                                    .user_ctx = NULL};

/** URI handler for all GET requests for /current_config.txt */
static httpd_uri_t uri_cfg_get = {.uri = "/current_config.txt",
                                  .method = HTTP_GET,
                                  .handler = &config_server_get_config_handler,
                                  .user_ctx = NULL};

/** URI handler for all POST requests for / */
static httpd_uri_t uri_post = {.uri = "/",
                               .method = HTTP_POST,
                               .handler = &config_server_post_handler,
                               .user_ctx = NULL};

/** Active HTTP server. */
httpd_handle_t http_server = NULL;

/** Tag used in logging. */
static const char *TAG = "config-server";

/* Public function implementation ------------------------------------------- */

bool config_server_start(void) {
    // Start from default configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    esp_err_t err;
    if ((err = httpd_start(&http_server, &config) == ESP_OK)) {

        err = httpd_register_uri_handler(http_server, &uri_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler (%d)", err);
            return false;
        }

        err = httpd_register_uri_handler(http_server, &uri_post);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add POST handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_style_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_cfg_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler  (%d)", err);
        }

    } else {
        ESP_LOGE(TAG, "Failed to start! (%d).", err);
        return false;
    }

    return true;
}

/* Private function implementation ------------------------------------------ */

static esp_err_t config_server_get_handler(httpd_req_t *req) {
    // Currently sends a hard-coded response.
    httpd_resp_send(req, (const char *)configserver_start,
                    configserver_end - configserver_start);
    return ESP_OK;
}

static esp_err_t config_server_get_style_handler(httpd_req_t *req) {
    // Currently sends a hard-coded response.
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_start,
                    style_end - style_start);
    return ESP_OK;
}

static esp_err_t config_server_post_handler(httpd_req_t *req) {
    // Buffer for received data (could also be binary). Assume it is a string.
    // Actual length of data will be returned by req_recv(), and then
    // zero-termination is ensured. If it is binary, the things below will just
    // parse garbage, but that's okay.
    char query_string[1000];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = (req->content_len < sizeof(query_string))
                           ? req->content_len
                           : sizeof(query_string);

    int ret = httpd_req_recv(req, query_string, recv_size);
    if (ret <= 0) {
        // connection closed for some reason
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            // If client times out, we just send timeout notice
            httpd_resp_send_408(req);
        }
        // return ESP_FAIL to close underlying socket
        return ESP_FAIL;
    }

    // Zero-terminate to be nice
    query_string[ret] = '\0';
    ESP_LOGI(TAG, "Rx POST req: '%s'", query_string);

    // Received something like ip=[...]&port=[...]
    // Use ESP-IDF functionality to fetch the value for each defined attribute,
    // decode it (may%20be%20encoded), then pass to
    // config_server_update_config().

    // Attributes that are currently supported in POST requests
    const char *attribute_keys[] = {
        "agent_ip", "agent_port",
        "ssid", "pwd", "static_ip", "static_netmask", "static_gw"
    };

    // Value of the attribute. Limited to 128 bytes including nul-termination.
    char value[128];
    char decoded_value[128]; // holds the value of the attribute after decoding
    bool should_commit_nvs = false; // whether the NVS needs a commit()
    size_t n = sizeof(attribute_keys) / sizeof(attribute_keys[0]);
    for (int i = 0; i < n; i++) {
        esp_err_t ret = httpd_query_key_value(query_string, attribute_keys[i],
                                              value, sizeof(value));
        if (ret == ESP_ERR_NOT_FOUND) {
            continue;
        } else if (ret == ESP_ERR_HTTPD_RESULT_TRUNC) {
            ESP_LOGW(TAG,
                     "HTTP Server buffer is not long enough to hold passed "
                     "argument. Ignoring.");
            continue;
        } else if (ret == ESP_OK) {
            // something found, maybe time to update?
            url_decode(decoded_value, value);
            if (config_server_update_config(attribute_keys[i], decoded_value))
                should_commit_nvs = true;
        }
    }

    if (should_commit_nvs) {
        nvs_store_config();
        const char resp[] = "Received and saved data.";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    } else {
        const char resp[] = "Nothing to update.";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }

    return ESP_OK;
}

static esp_err_t config_server_get_config_handler(httpd_req_t *req) {
    char response[1000];
    snprintf(
        response, sizeof(response),
        "agent_ip=%s\nagent_port=%d\nssid=%s\npwd=%s\nstatic_ip=%s\nstatic_netmask=%s\nstatic_gw=%s\nsoftware_version=%s",
        global_config.agent_ip, global_config.agent_port,
        global_config.ssid, global_config.pwd,
        global_config.static_ip, global_config.static_netmask, global_config.static_gw, GIT_TAG);
    httpd_resp_send(req, (const char *)response, strlen(response));
    return ESP_OK;
}

// The following function was taken from StackOverflow
// Source: https://stackoverflow.com/a/14530993
void url_decode(char *dst, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') && ((a = src[1]) && (b = src[2])) &&
            (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a')
                a -= 'a' - 'A';
            if (a >= 'A')
                a -= ('A' - 10);
            else
                a -= '0';
            if (b >= 'a')
                b -= 'a' - 'A';
            if (b >= 'A')
                b -= ('A' - 10);
            else
                b -= '0';
            *dst++ = 16 * a + b;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst++ = '\0';
}

static bool config_server_update_config(const char *key, const char *value) {
    // Reject empty fields
    if (!key || !value || value[0] == '\0')
        return false;

    if (strncmp(key, "agent_ip", 8) == 0) {
        // update global IP attribute
        // copy value into buffer, zero-termination by strlcpy
        strlcpy(global_config.agent_ip, value, sizeof(global_config.agent_ip));
        return true;
    } else if (strncmp(key, "agent_port", 10) == 0) {
        // update global port attribute
        // copy value into buffer, zero-termination by strlcpy
        global_config.agent_port = atoi(value);
        return true;
    } else if (strncmp(key, "ssid", 4) == 0) {
        // update global ssid attribute
        // copy value into buffer, zero-termination by strlcpy
        strlcpy(global_config.ssid, value, sizeof(global_config.ssid));
        return true;
    } else if (strncmp(key, "pwd", 3) == 0) {
        // update glboal pwd attribute
        // copy value into buffer, zero-termination by strlcpy
        strlcpy(global_config.pwd, value, sizeof(global_config.pwd));
        return true;
    } else if (strncmp(key, "static_ip", 9) == 0) {
        strlcpy(global_config.static_ip, value, sizeof(global_config.static_ip));
        return true;
    } else if (strncmp(key, "static_netmask", 14) == 0) {
        strlcpy(global_config.static_netmask, value, sizeof(global_config.static_netmask));
        return true;
    } else if (strncmp(key, "static_gw", 9) == 0) {
        strlcpy(global_config.static_gw, value, sizeof(global_config.static_gw));
        return true;
    }
    return false;
}
