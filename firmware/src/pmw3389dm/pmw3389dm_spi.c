#include "pmw3389dm_spi.h"
#include "pmw3389dm_srom.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"

#define PMW3389DM_CLK_FREQ         (2 * 1000 * 1000)   // 2MHz
#define PMW3389DM_INPUT_DELAY_NS   (90) // 90ns

#define PRODUCT_ID 0x00
#define MOTION 0x02
#define DELTA_X_L 0x03
#define DELTA_X_H 0x04
#define DELTA_Y_L 0x05
#define DELTA_Y_H 0x06
#define RESOLUTION_L 0x0E
#define RESOLUTION_H 0x0F
#define CONFIG2 0x10
#define SROM_ENABLE 0x13
#define SROM_ID 0x2A
#define POWER_UP_RESET 0x3A
#define SHUTDOWN  0x3B
#define INVERSE_PRODUCT_ID 0x3F
#define MOTION_BURST 0x50
#define SROM_LOAD_BURST 0x62

/// Context (config and data) of the spi_pmw3389dm
struct pmw3389dm_context_t {
    pmw3389dm_config_t cfg;      ///< Configuration by the caller.
    spi_device_handle_t spi;     ///< SPI device handle
    SemaphoreHandle_t ready_sem; ///< Semaphore for ready signal
};

typedef struct pmw3389dm_context_t pmw3389dm_context_t;

static const char TAG[] = "PMW3389DM";

static void com_begin(pmw3389dm_context_t* ctx)
{
    gpio_set_level(ctx->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ctx->cfg.cs_io);
}

static void com_end(pmw3389dm_context_t* ctx)
{
    gpio_set_level(ctx->cfg.cs_io, 1);
    ESP_EARLY_LOGV(TAG, "cs high %d.", ctx->cfg.cs_io);
}

esp_err_t spi_pmw3389dm_init(const pmw3389dm_config_t *cfg, pmw3389dm_context_t** out_ctx)
{
    esp_err_t err = ESP_OK;
    pmw3389dm_context_t* ctx = (pmw3389dm_context_t*)malloc(sizeof(pmw3389dm_context_t));
    if (!ctx) return ESP_ERR_NO_MEM;

    *ctx = (pmw3389dm_context_t) {
            .cfg = *cfg,
    };

    spi_device_interface_config_t devcfg={
            .clock_speed_hz = PMW3389DM_CLK_FREQ,
            .mode = 3, //SPI mode 3
            .spics_io_num = -1, // manual CS management
            .queue_size = 1,
            .input_delay_ns = PMW3389DM_INPUT_DELAY_NS,
    };
    //Attach the PMW3389DM to the SPI bus
    err = spi_bus_add_device(ctx->cfg.host, &devcfg, &ctx->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(ctx->cfg.rs_io, 1);
    gpio_config_t rs_cfg = {
            .pin_bit_mask = BIT64(ctx->cfg.rs_io),
            .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&rs_cfg);

    gpio_set_level(ctx->cfg.cs_io, 0);
    gpio_config_t cs_cfg = {
            .pin_bit_mask = BIT64(ctx->cfg.cs_io),
            .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&cs_cfg);

    *out_ctx = ctx;
    return ESP_OK;

    cleanup:
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    if (ctx->ready_sem) {
        vSemaphoreDelete(ctx->ready_sem);
        ctx->ready_sem = NULL;
    }
    free(ctx);
    return err;
}

esp_err_t spi_pmw3389dm_deinit(pmw3389dm_context_t* ctx)
{
    spi_bus_remove_device(ctx->spi);
    free(ctx);
    return ESP_OK;
}

esp_err_t spi_pmw3389dm_read(pmw3389dm_context_t* ctx, uint8_t address, uint8_t* out_data)
{
    com_begin(ctx);

    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // zero out transaction
    t.length = 8;
    t.tx_data[0] = address & 0x7f;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.user = ctx;

    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    usleep(160); // t_SRAD 160us

    t.length = 8;
    t.tx_buffer = NULL;
    t.rx_buffer = out_data;

    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    spi_device_release_bus(ctx->spi);

    usleep(1); // t_SCLK-NCS (read) 120ns
    com_end(ctx);
    usleep(19); // t_SRW/SRR 20us - tSCLK-NCS

    return err;
}

esp_err_t spi_pmw3389dm_write(pmw3389dm_context_t* ctx, uint8_t address, uint8_t data)
{
    com_begin(ctx);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // zero out transaction
    t.length = 16;
    t.tx_data[0] = address | 0x80;
    t.tx_data[1] = data;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.user = ctx;

    esp_err_t err = spi_device_polling_transmit(ctx->spi, &t);

    usleep(35); // t_SCLK-NCS (write) 35us
    com_end(ctx);
    usleep(145); // t_SWW/SWR 180us - t_SCLK-NCS

    return err;
}

esp_err_t spi_pmw3389dm_power_up_and_upload_firmware(pmw3389dm_context_t* ctx)
{
    ESP_LOGI(TAG, "Performing Power-Up for PMW3389DM...");

    // toggle ncs to reset SPI port
    com_end(ctx);
    com_begin(ctx);
    com_end(ctx);

    esp_err_t err;

    // shutdown sensor
    err = spi_pmw3389dm_write(ctx, SHUTDOWN, 0xB6);
    if (err != ESP_OK) return err;

    // wait 300ms
    usleep(300 * 1000);

    // toggle ncs to reset SPI port
    com_begin(ctx);
    usleep(40);
    com_end(ctx);
    usleep(40);

    // reset register
    err = spi_pmw3389dm_write(ctx, POWER_UP_RESET, 0x5A);
    if (err != ESP_OK) return err;

    // wait 50ms
    usleep(50 * 1000);

    // read registers once
    uint8_t data;
    err = spi_pmw3389dm_read(ctx, MOTION, &data);
    if (err != ESP_OK) return err;
    err = spi_pmw3389dm_read(ctx, DELTA_X_L, &data);
    if (err != ESP_OK) return err;
    err = spi_pmw3389dm_read(ctx, DELTA_X_H, &data);
    if (err != ESP_OK) return err;
    err = spi_pmw3389dm_read(ctx, DELTA_Y_L, &data);
    if (err != ESP_OK) return err;
    err = spi_pmw3389dm_read(ctx, DELTA_Y_H, &data);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Uploading Firmware to PMW3389DM...");

    // disable reset mode
    err = spi_pmw3389dm_write(ctx, CONFIG2, 0x20);
    if (err != ESP_OK) return err;
    // initialize SROM download
    err = spi_pmw3389dm_write(ctx, SROM_ENABLE, 0x1D);
    if (err != ESP_OK) return err;
    // wait 10ms
    usleep(10 * 1000);
    // start SROM downlaod
    err = spi_pmw3389dm_write(ctx, SROM_ENABLE, 0x18);
    if (err != ESP_OK) return err;

    // write SROM
    com_begin(ctx);

    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // zero out transaction
    t.length = 8;
    t.tx_data[0] = SROM_LOAD_BURST | 0x80;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.user = ctx;
    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    usleep(15);

    for (int i = 0; i < firmware_length; i++) {
        t.tx_data[0] = firmware_data[i];
        err = spi_device_polling_transmit(ctx->spi, &t);
        if (err != ESP_OK) return err;

        usleep(15);
    }

    spi_device_release_bus(ctx->spi);

    // exit bust mode
    com_end(ctx);

    err = spi_pmw3389dm_read(ctx, SROM_ID, &data);
    if (err != ESP_OK) return err;

    err = spi_pmw3389dm_write(ctx, CONFIG2, 0x00);
    if (err != ESP_OK) return err;

    usleep(10 * 1000);

    ESP_LOGI(TAG, "PMW3389DM initialized");

    err = spi_pmw3389dm_read(ctx, PRODUCT_ID, &data);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Product_ID: 0x%02X", data);
    err = spi_pmw3389dm_read(ctx, INVERSE_PRODUCT_ID, &data);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Inverse_Product_ID: 0x%02X", data);
    err = spi_pmw3389dm_read(ctx, SROM_ID, &data);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "SROM_Version: 0x%02X", data);
    err = spi_pmw3389dm_read(ctx, MOTION, &data);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Motion: 0x%02X", data);

    return err;
}

esp_err_t spi_pmw3389dm_init_burst_read(pmw3389dm_context_t* ctx)
{
    return spi_pmw3389dm_write(ctx, MOTION_BURST, 0x01);
}

esp_err_t spi_pmw3389dm_burst_read(pmw3389dm_context_t* ctx, uint8_t* out_data)
{
    com_begin(ctx);

    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // zero out transaction
    t.length = 8;
    t.tx_data[0] = MOTION_BURST;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.user = ctx;

    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    usleep(35); // t_SRAD_MOTBR 35us

    // for some reason only chunks of 4 bytes work
    t.length = 4 * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = out_data;
    t.flags = 0;

    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    t.length = 4 * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = out_data + 4;
    t.flags = 0;

    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    t.length = 4 * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = out_data + 8;
    t.flags = 0;

    err = spi_device_polling_transmit(ctx->spi, &t);
    if (err != ESP_OK) return err;

    spi_device_release_bus(ctx->spi);

    com_end(ctx);

    return err;
}

esp_err_t spi_pmw3389dm_set_cpi(pmw3389dm_context_t* ctx, uint16_t cpi)
{
    uint16_t cpi_val = cpi / 50;

    esp_err_t err;
    err = spi_pmw3389dm_write(ctx, RESOLUTION_L, (cpi_val & 0xFF));
    if (err != ESP_OK) return err;
    err = spi_pmw3389dm_write(ctx, RESOLUTION_H, ((cpi_val >> 8) & 0xFF));
    return err;
}
