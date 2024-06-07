#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "spideve.h"

#include "pmw3389dm.hpp"
#include "pmw3389dm_srom.hpp"

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

PMW3389DM::PMW3389DM() : chip(nullptr), line(nullptr), fd(-1) {}

PMW3389DM::~PMW3389DM()
{
    stop();
}

bool PMW3389DM::begin(const char* device, const char* n_reset_line_name)
{
    line = gpiod_line_find(n_reset_line_name);
    if (line == nullptr) {
        std::cerr << "Failed to find line: " << n_reset_line_name << std::endl;
        return false;
    }
    chip = gpiod_line_get_chip(line);

    return begin_common(device);
}

bool PMW3389DM::begin(const char* device, const char* n_reset_chipname, unsigned int n_reset_line_offset)
{
    chip = gpiod_chip_open_by_name(n_reset_chipname);
    if (chip == nullptr) {
        std::cerr << "Failed to open chip: " << n_reset_chipname << std::endl;
        return false;
    }
    line = gpiod_chip_get_line(chip, n_reset_line_offset);
    if (line == nullptr) {
        std::cerr << "Failed to open line: " << n_reset_line_offset << std::endl;
        gpiod_chip_close(chip);
        chip = nullptr;
        return false;
    }

    return begin_common(device);
}

bool PMW3389DM::begin_common(const char* device)
{
    // set n_reset pin to 1 (no reset)
    if (gpiod_line_request_output(line, "pmw3389dm", 1) < 0) {
        std::cerr << "Failed to reserve n_reset pin" << std::endl;
        gpiod_chip_close(chip);
        chip = nullptr;
        line = nullptr;
        return false;
    }

    fd = open(device, O_RDWR);
    if (fd == -1) {
        std::cerr << "Failed to open port: " << device << std::endl;
        stop();
        return false;
    }

    uint8_t mode = SPI_MODE_3;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode" << std::endl;
        stop();
        return false;
    }

    uint8_t bits_per_word = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        std::cerr << "Failed to set SPI bits per word" << std::endl;
        stop();
        return false;
    }

    uint32_t speed = 2000000;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Failed to set SPI max speed" << std::endl;
        stop();
        return false;
    }

    return true;
}

void PMW3389DM::stop()
{
    if (line != nullptr) {
        gpiod_line_release(line);
        line = nullptr;
    }
    if (chip != nullptr) {
        gpiod_chip_close(chip);
        chip = nullptr;
    }
    if (fd != -1) {
        close(fd);
    }
}

int PMW3389DM::read_reg(uint8_t address, uint8_t* rx_buf)
{
    struct spi_ioc_transfer xfer[2];
    memset(xfer, 0, sizeof(xfer));

    uint8_t tx_data = address & 0x7f;

    xfer[0].tx_buf = (__u64) &tx_data;
    xfer[0].rx_buf = (__u64) 0;
    xfer[0].len = 1;
    xfer[0].delay_usecs = 160; // t_SRAD 160us

    xfer[1].tx_buf = (__u64) 0;
    xfer[1].rx_buf = (__u64) rx_buf;
    xfer[1].len = 1;
    xfer[1].delay_usecs = 1; // t_SCLK-NCS (read) 120ns

    int retv = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);

    usleep(20); // t_SRW/SRR 20us - tSCLK-NCS

    return retv;
}

int PMW3389DM::write_reg(uint8_t address, uint8_t data)
{
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));

    uint8_t tx_data[2];
    tx_data[0] = address | 0x80;
    tx_data[1] = data;

    xfer.tx_buf = (__u64) &tx_data;
    xfer.rx_buf = (__u64) 0;
    xfer.len = 2;
    xfer.delay_usecs = 35; // t_SCLK-NCS (write) 35us

    int retv = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);

    usleep(145); // t_SWW/SWR 180us - t_SCLK-NCS

    return retv;
}

int PMW3389DM::power_up_and_upload_firmware()
{
    int retv;
    std::cout << "Performing Power-Up for PMW3389DM..." << std::endl;

    // shutdown sensor
    retv = write_reg(SHUTDOWN, 0xB6);
    if (retv < 0) {
        std::cerr << "Error shutting down sensor" << std::endl;
        return retv;
    }

    // wait 300ms
    usleep(300 * 1000);

    // reset register
    retv = write_reg(POWER_UP_RESET, 0x5A);
    if (retv < 0) {
        std::cerr << "Error resetting sensor" << std::endl;
        return retv;
    }

    // wait 50ms
    usleep(50 * 1000);

    // read registers once
    uint8_t data;
    retv = read_reg(MOTION, &data);
    if (retv < 0) return retv;
    retv = read_reg(DELTA_X_L, &data);
    if (retv < 0) return retv;
    retv = read_reg(DELTA_X_H, &data);
    if (retv < 0) return retv;
    retv = read_reg(DELTA_Y_L, &data);
    if (retv < 0) return retv;
    retv = read_reg(DELTA_Y_H, &data);
    if (retv < 0) return retv;

    std::cout << "Uploading Firmware to PMW3389DM..." << std::endl;

    // disable reset mode
    retv = write_reg(CONFIG2, 0x20);
    if (retv < 0) return retv;
    // initialize SROM download
    retv = write_reg(SROM_ENABLE, 0x1D);
    if (retv < 0) return retv;
    // wait 10ms
    usleep(10 * 1000);
    // start SROM downlaod
    retv = write_reg(SROM_ENABLE, 0x18);
    if (retv < 0) return retv;

    struct spi_ioc_transfer xfer[firmware_length + 1];
    memset(xfer, 0, sizeof(xfer));

    uint8_t tx_data = SROM_LOAD_BURST | 0x80;
    xfer[0].tx_buf = (__u64) &tx_data;
    xfer[0].rx_buf = (__u64) 0;
    xfer[0].len = 1;
    xfer[0].delay_usecs = 15;

    for (int i = 0; i < firmware_length; i++) {
        xfer[i + 1].tx_buf = (__u64) (firmware_data + i);
        xfer[i + 1].rx_buf = (__u64) 0;
        xfer[i + 1].len = 1;
        xfer[i + 1].delay_usecs = 15;
    }

    retv = ioctl(fd, SPI_IOC_MESSAGE_E(firmware_length + 1), xfer);
    if (retv < 0) {
        std::cerr << "Failed uploading firmware" << std::endl;
        return retv;
    }

    retv = read_reg(SROM_ID, &data);
    if (retv < 0) return retv;

    retv = write_reg(CONFIG2, 0x00);
    if (retv < 0) return retv;

    usleep(10 * 1000);

    std::cout << "PMW3389DM initialized" << std::endl;

    retv = read_reg(PRODUCT_ID, &data);
    if (retv < 0) return retv;
    printf("Product_ID: 0x%02X\n", data);
    retv = read_reg(INVERSE_PRODUCT_ID, &data);
    if (retv < 0) return retv;
    printf("Inverse_Product_ID: 0x%02X\n", data);
    retv = read_reg(SROM_ID, &data);
    if (retv < 0) return retv;
    printf("SROM_Version: 0x%02X\n", data);
    retv = read_reg(MOTION, &data);
    if (retv < 0) return retv;
    printf("Motion: 0x%02X\n", data);
    fflush(stdout);

    return retv;
}
