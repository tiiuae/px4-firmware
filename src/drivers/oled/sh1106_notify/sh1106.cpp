#include "sh1106.hpp"
#include "font6x8.hpp"
#include <string.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_defines.h> // PX4_WARN, PX4_ERR

// SH1106 commands (subset)
static constexpr uint8_t CMD_DISPLAY_OFF  = 0xAE;
static constexpr uint8_t CMD_DISPLAY_ON   = 0xAF;
static constexpr uint8_t CMD_SET_DISP_CLK = 0xD5;
static constexpr uint8_t CMD_SET_MUX      = 0xA8;
static constexpr uint8_t CMD_SET_OFFSET   = 0xD3;
static constexpr uint8_t CMD_SET_START    = 0x40;
static constexpr uint8_t CMD_CHARGE_PUMP  = 0x8D; // not always used on SH1106
static constexpr uint8_t CMD_SET_ADDRING  = 0x20; // horiz/vert addressing (mostly SSD1306)
static constexpr uint8_t CMD_SEG_REMAP    = 0xA1;
static constexpr uint8_t CMD_COM_SCAN_DEC = 0xC8;
static constexpr uint8_t CMD_SET_CONTRAST = 0x81;
static constexpr uint8_t CMD_SET_PRECHRG  = 0xD9;
static constexpr uint8_t CMD_SET_VCOM     = 0xDB;
static constexpr uint8_t CMD_ENTIRE_ON    = 0xA4;
static constexpr uint8_t CMD_NORMAL_DISP  = 0xA6;

SH1106::SH1106(int bus, int addr) : I2C(DRV_DEVICE_PATH("sh1106", bus), "sh1106", bus, addr, 400000) {}

int SH1106::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) { PX4_ERR("I2C init failed"); return ret; }

    // Init sequence (typical SH1106)
    const uint8_t init_seq[] = {
        CMD_DISPLAY_OFF,
        CMD_SET_DISP_CLK, 0x80,
        CMD_SET_MUX, 0x3F,            // 64 mux
        CMD_SET_OFFSET, 0x00,
        CMD_SET_START | 0x00,         // start line = 0
        CMD_SEG_REMAP,                // mirror X (depends orientation)
        CMD_COM_SCAN_DEC,             // scan from COM[N-1] to COM0
        CMD_SET_CONTRAST, 0x7F,
        CMD_ENTIRE_ON,
        CMD_NORMAL_DISP,
        CMD_SET_PRECHRG, 0xF1,
        CMD_SET_VCOM, 0x40,
        CMD_DISPLAY_ON
    };
    ret = send_cmds(init_seq, sizeof(init_seq));
    if (ret != PX4_OK) return ret;

    clear();
    return PX4_OK;
}

void SH1106::contrast(uint8_t c) {
    const uint8_t cmds[] = { CMD_SET_CONTRAST, c };
    send_cmds(cmds, sizeof(cmds));
}

void SH1106::clear() {
    for (uint8_t page = 0; page < 8; page++) {
        set_page_col(page, 0);
        uint8_t zeros[128] {};
        send_data(zeros, sizeof(zeros));
    }
    set_cursor(0,0);
}

void SH1106::set_cursor(uint8_t col, uint8_t row) {
    _col = col; _row = row;
}

void SH1106::print(const char *s) {
    while (*s) {
        char c = *s++;
        if (c == '\n') { _row++; _col = 0; continue; }
        if (c < 0x20 || c > 0x7F) c = '?';
        const uint8_t *ch = g_font6x8[c - 0x20];
        set_page_col(_row, _col * 6);
        send_data(ch, 6);
        _col++;
        if (_col >= 21) { _col = 0; _row++; }
        if (_row >= 8) break;
    }
}

void SH1106::print_line(uint8_t row, const char *s) {
    // clear the line first (21 chars * 6px = 126, write 128)
    set_page_col(row, 0);
    uint8_t buf[128]; memset(buf, 0x00, sizeof(buf));
    // draw into a temp line buffer
    uint8_t x = 0;
    while (*s && x < 21) {
        char c = *s++;
        if (c < 0x20 || c > 0x7F) c = '?';
        const uint8_t *ch = g_font6x8[c - 0x20];
        memcpy(&buf[x*6], ch, 6);
        x++;
    }
    send_data(buf, sizeof(buf));
}

int SH1106::send_cmd(uint8_t c) {
    uint8_t buf[2] = {0x00, c}; // 0x00 = command
    return transfer(&buf[0], sizeof(buf), nullptr, 0);
}

int SH1106::send_cmds(const uint8_t *cmds, size_t n) {
    int ret = PX4_OK;
    for (size_t i = 0; i < n; i++) {
        ret = send_cmd(cmds[i]);
        if (ret != PX4_OK) break;
    }
    return ret;
}

int SH1106::send_data(const uint8_t *data, size_t n) {
    // 0x40 = data
    // Write in chunks that fit the I2C DMA/stack
    while (n) {
        size_t chunk = n;
        if (chunk > 16) chunk = 16;
        uint8_t tmp[1 + 16];
        tmp[0] = 0x40;
        memcpy(&tmp[1], data, chunk);
        int ret = transfer(tmp, 1 + chunk, nullptr, 0);
        if (ret != PX4_OK) return ret;
        data += chunk; n -= chunk;
    }
    return PX4_OK;
}

void SH1106::set_page_col(uint8_t page, uint8_t col) {
    // SH1106 uses page addressing; columns 0..127
    send_cmd(0xB0 | (page & 0x07));                 // set page
    send_cmd(0x00 | (col & 0x0F));                  // lower nibble
    send_cmd(0x10 | ((col >> 4) & 0x0F));           // upper nibble
}
