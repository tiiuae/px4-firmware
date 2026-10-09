/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bootloader_main.c
 *
 * FMU-specific early startup code for bootloader
*/

#include "board_config.h"
#include "hw_config.h"
#include "bl.h"

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <chip.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include <px4_platform_common/init.h>
#include <hardware/imxrt_flexspi.h>
#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>
#include <nuttx/cache.h>
#include <nuttx/mtd/mtd.h>
#include <string.h>

#include "slots.h"

#define FRAM_BLOCK_SIZE 128

extern struct flexspi_nor_config_s g_bootConfig;

static struct mtd_dev_s *g_fram;

extern int sercon_main(int c, char **argv);

void board_late_initialize(void)
{
	sercon_main(0, NULL);
}

extern void sys_tick_handler(void);
void board_timerhook(void)
{
	sys_tick_handler();
}

static struct mtd_dev_s *fram(void)
{
	if (g_fram == NULL && imxrt_flexspi_fram_initialize() == OK) {
		g_fram = imxrt_flexspi_fram_mtd();
	}

	return g_fram;
}

int board_devstate_read(uint8_t *buf, size_t size)
{
	uint8_t block[FRAM_BLOCK_SIZE];

	if (fram() == NULL || MTD_BREAD(g_fram, BOARD_FRAM_DEVSTATE_BLOCK, 1, block) != 1) {
		return -1;
	}

	memcpy(buf, block, size < sizeof(block) ? size : sizeof(block));
	return size < sizeof(block) ? size : sizeof(block);
}

int board_devstate_write(const uint8_t *buf, size_t size)
{
	uint8_t block[FRAM_BLOCK_SIZE];

	if (fram() == NULL || size > sizeof(block)) {
		return -1;
	}

	memset(block, 0xff, sizeof(block));
	memcpy(block, buf, size);
	return MTD_BWRITE(g_fram, BOARD_FRAM_DEVSTATE_BLOCK, 1, block) == 1 ? 0 : -1;
}

static uintptr_t slot_offset(int slot)
{
	return (APP_LOAD_ADDRESS - IMXRT_FLEXSPI1_CIPHER_BASE) + (slot ? BOARD_SLOT_B_OFFSET : 0);
}

bool board_slot_bootable(int slot)
{
	const uint32_t *vectors = (const uint32_t *)(IMXRT_FLEXSPI1_CIPHER_BASE + slot_offset(slot) + APP_VECTOR_OFFSET);

	return vectors[0] != 0xffffffff;
}

locate_code(".ramfunc")
void board_slot_erase(int slot)
{
	irqstate_t flags = enter_critical_section();
	ROM_FLEXSPI_NorFlash_Erase(1, &g_bootConfig, slot_offset(slot) + APP_VECTOR_OFFSET, 4 * 1024);
	ROM_FLEXSPI_NorFlash_ClearCache(1);
	leave_critical_section(flags);
	up_invalidate_dcache_all();
}

void board_slot_select(int slot)
{
	if (slot) {
		putreg32(BOARD_SLOT_B_OFFSET, IMXRT_FLEXSPI1_HADDROFFSET);
		putreg32(IMXRT_FLEXSPI1_CIPHER_BASE + BOARD_FLASH_SIZE, IMXRT_FLEXSPI1_HADDREND);
		putreg32(APP_LOAD_ADDRESS | 1, IMXRT_FLEXSPI1_HADDRSTART);

	} else {
		putreg32(0, IMXRT_FLEXSPI1_HADDRSTART);
	}

	ROM_FLEXSPI_NorFlash_ClearCache(1);
	up_invalidate_dcache_all();
	up_invalidate_icache_all();
}
