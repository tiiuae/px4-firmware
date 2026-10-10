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
#include "imxrt_flexspi_nor_boot.h"

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
#include <stddef.h>
#include <string.h>

#include "slots.h"

#define FRAM_BLOCK_SIZE 128

#define HAB_TAG_RVT     0xdd
#define HAB_SUCCESS     0xf0
#define HAB_STS_ANY     0x00
#define HAB_CID_CALLER  1
#define HAB_EVENT_MAX   32
#define HAB_MARK        0x48414231

struct hab_rvt_s {
	uint32_t hdr;
	uint8_t (*entry)(void);
	uint8_t (*exit)(void);
	uint8_t (*check_target)(uint8_t type, const void *start, size_t bytes);
	void *(*authenticate_image)(uint8_t cid, ptrdiff_t ivt_offset, void **start, size_t *bytes, void *loader);
	uint8_t (*run_dcd)(const uint8_t *dcd);
	uint8_t (*run_csf)(const uint8_t *csf, uint8_t cid, uint32_t srkmask);
	uint8_t (*assert_check)(uint8_t type, const void *data, uint32_t count);
	uint8_t (*report_event)(uint8_t status, uint32_t index, uint8_t *event, size_t *bytes);
	uint8_t (*report_status)(uint8_t *config, uint8_t *state);
	void (*failsafe)(void);
};

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

#if !defined(BOARD_HAB_CSF_OFFSET)
bool board_slot_verify(void)
{
	return true;
}

#else
static const struct hab_rvt_s *hab_rvt(void)
{
	static const uint32_t cand[] = {0x00211c0c, 0x00211c14};

	for (unsigned i = 0; i < sizeof(cand) / sizeof(cand[0]); i++) {
		const struct hab_rvt_s *rvt = (const struct hab_rvt_s *)cand[i];

		if ((rvt->hdr & 0xff) == HAB_TAG_RVT) {
			return rvt;
		}
	}

	return NULL;
}

locate_code(".ramfunc")
static uint32_t hab_events(const struct hab_rvt_s *rvt)
{
	uint32_t n = 0;
	size_t bytes = 0;

	while (n < HAB_EVENT_MAX && rvt->report_event(HAB_STS_ANY, n, NULL, &bytes) == HAB_SUCCESS) {
		n++;
	}

	return n;
}

locate_code(".ramfunc")
static bool hab_accepts_slot(const struct hab_rvt_s *rvt)
{
	void *start = (void *)APP_LOAD_ADDRESS;
	size_t bytes = BOARD_SLOT_SIZE;
	irqstate_t flags = enter_critical_section();
	uint32_t before = hab_events(rvt);

	rvt->entry();
	void *entry = rvt->authenticate_image(HAB_CID_CALLER, APP_IVT_OFFSET, &start, &bytes, NULL);
	rvt->exit();

	bool clean = entry != NULL && hab_events(rvt) == before;

	ROM_FLEXSPI_NorFlash_ClearCache(1);
	leave_critical_section(flags);
	return clean;
}

static bool hab_mark_set(void)
{
	uint8_t block[FRAM_BLOCK_SIZE];
	uint32_t mark;

	if (fram() == NULL || MTD_BREAD(g_fram, BOARD_FRAM_HAB_BLOCK, 1, block) != 1) {
		return false;
	}

	memcpy(&mark, block, sizeof(mark));
	return mark == HAB_MARK;
}

static void hab_mark_write(uint32_t mark)
{
	uint8_t block[FRAM_BLOCK_SIZE];

	if (fram() == NULL) {
		return;
	}

	memset(block, 0xff, sizeof(block));
	memcpy(block, &mark, sizeof(mark));
	MTD_BWRITE(g_fram, BOARD_FRAM_HAB_BLOCK, 1, block);
}

static bool slot_claims_signature(void)
{
	const uint32_t self = APP_LOAD_ADDRESS + APP_IVT_OFFSET;
	const uint32_t *ivt = (const uint32_t *)self;

	if ((ivt[0] & 0xff) != IVT_TAG_HEADER || ivt[5] != self || ivt[4] != self + 0x20) {
		return false;
	}

	if (ivt[6] <= self || ivt[6] >= APP_LOAD_ADDRESS + BOARD_SLOT_SIZE) {
		return false;
	}

	const uint32_t *bdata = (const uint32_t *)ivt[4];

	return bdata[0] == APP_LOAD_ADDRESS && bdata[1] <= BOARD_SLOT_SIZE;
}

bool board_slot_verify(void)
{
	const struct hab_rvt_s *rvt = hab_rvt();

	if (rvt == NULL || !slot_claims_signature()) {
		return false;
	}

	if (hab_mark_set()) {
		hab_mark_write(0);
		return true;
	}

	hab_mark_write(HAB_MARK);
	up_flush_dcache_all();
	bool clean = hab_accepts_slot(rvt);
	up_invalidate_dcache_all();
	up_invalidate_icache_all();
	hab_mark_write(0);
	return clean;
}
#endif

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
