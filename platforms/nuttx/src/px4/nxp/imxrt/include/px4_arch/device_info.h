/*
 * Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 */

#pragma once

#include <stdint.h>
#include <image_toc.h>

/* Bootloader ABI: keep the same layout and padding as the other Saluki boards. */

typedef struct __attribute__((__packed__))
{
	uint32_t magic;
	uint16_t arb[IMAGE_NUM_TYPES];
	uint32_t features;
	uint8_t mac[4][6];
	char bl_version[32];
	uint32_t fpga_version;
	uint32_t boot_reason;
	uint8_t hw_version;
	uint8_t hw_revision;
	uint8_t board_version;
	uint8_t board_revision;
	uint8_t reserved[416];
} devinfo_t;

#ifdef __cplusplus
extern "C" {
#endif

extern devinfo_t device_boot_info;

#ifdef __cplusplus
}
#endif
