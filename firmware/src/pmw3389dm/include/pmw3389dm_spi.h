#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_pmw3389dm_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `spi_pmw3389dm_init()`
    gpio_num_t rs_io;       ///< RS gpio number, set before calling `spi_pmw3389dm_init()`
} pmw3389dm_config_t;

typedef struct pmw3389dm_context_t* pmw3389dm_handle_t;

esp_err_t spi_pmw3389dm_init(const pmw3389dm_config_t *cfg, pmw3389dm_handle_t* out_ctx);
esp_err_t spi_pmw3389dm_deinit(pmw3389dm_handle_t ctx);

esp_err_t spi_pmw3389dm_read(pmw3389dm_handle_t ctx, uint8_t address, uint8_t* out_data);
esp_err_t spi_pmw3389dm_write(pmw3389dm_handle_t ctx, uint8_t address, uint8_t data);

esp_err_t spi_pmw3389dm_power_up_and_upload_firmware(pmw3389dm_handle_t ctx);
esp_err_t spi_pmw3389dm_init_burst_read(pmw3389dm_handle_t ctx);
esp_err_t spi_pmw3389dm_burst_read(pmw3389dm_handle_t ctx, uint8_t* out_data);
esp_err_t spi_pmw3389dm_set_cpi(pmw3389dm_handle_t ctx, uint16_t cpi);
