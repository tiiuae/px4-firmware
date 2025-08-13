#pragma once
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <stdint.h>

// Default 128x64 SH1106 I2C address is usually 0x3C (sometimes 0x3D)
#ifndef SH1106_I2C_ADDR
# define SH1106_I2C_ADDR 0x3C
#endif

class SH1106 : public device::I2C
{
public:
    SH1106(int bus, int addr = SH1106_I2C_ADDR);
    ~SH1106() override = default;

    int init();
    void contrast(uint8_t c);
    void clear();
    void update_full(const uint8_t *buffer, size_t len);

    // Simple text drawing (6x8 font)
    void set_cursor(uint8_t col, uint8_t row);
    void print(const char *s);
    void print_line(uint8_t row, const char *s);

private:
    int send_cmd(uint8_t c);
    int send_cmds(const uint8_t *cmds, size_t n);
    int send_data(const uint8_t *data, size_t n);
    void set_page_col(uint8_t page, uint8_t col);

    uint8_t _col{0};
    uint8_t _row{0};
};
