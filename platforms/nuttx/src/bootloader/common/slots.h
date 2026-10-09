#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int board_devstate_read(uint8_t *buf, size_t size);
int board_devstate_write(const uint8_t *buf, size_t size);
bool board_slot_bootable(int slot);
void board_slot_erase(int slot);
void board_slot_select(int slot);

int slots_select(void);
void slots_reset(void);
