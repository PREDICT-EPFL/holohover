#ifndef HOLOHOVER_DRIVERS_PMW3389DM_HPP
#define HOLOHOVER_DRIVERS_PMW3389DM_HPP

#include <gpiod.h>

class PMW3389DM
{
public:
    PMW3389DM();
    ~PMW3389DM();

    bool begin(const char* device, const char* n_reset_line_name);
    bool begin(const char* device, const char* n_reset_chipname, unsigned int n_reset_line_offset);
    void stop();

    int read_reg(uint8_t address, uint8_t* rx_buf);
    int write_reg(uint8_t address, uint8_t data);
    int power_up_and_upload_firmware();

private:
    bool begin_common(const char* device);

    gpiod_chip* chip;
    gpiod_line* line;
    int fd;
};

#endif //HOLOHOVER_DRIVERS_PMW3389DM_HPP
