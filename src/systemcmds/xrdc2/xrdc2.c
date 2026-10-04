/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "imxrt_edma.h"
#include "hardware/imxrt_dmamux.h"
#include "hardware/rt117x/imxrt117x_xrdc2.h"

#define PROBE_LEN 16

struct copy_s {
	sem_t done;
	int result;
};

extern uint8_t _ssecmem[];
extern uint8_t _esecmem[];

__EXPORT int xrdc2_main(int argc, char *argv[]);

static uint8_t g_src[32] aligned_data(32);
static uint8_t g_dst[32] aligned_data(32);

static void copy_done(DMACH_HANDLE handle, void *arg, bool done, int result)
{
	struct copy_s *copy = arg;

	copy->result = result;
	nxsem_post(&copy->done);
}

static int dma_copy(const void *src)
{
	struct imxrt_edma_xfrconfig_s config;
	struct copy_s copy;
	DMACH_HANDLE ch;

	ch = imxrt_dmach_alloc(DMAMUX_CHCFG_ENBL | DMAMUX_CHCFG_AON, 0);

	if (ch == NULL) {
		return -EBUSY;
	}

	memset(g_dst, 0, sizeof(g_dst));
	up_clean_dcache((uintptr_t)g_dst, (uintptr_t)g_dst + sizeof(g_dst));

	memset(&config, 0, sizeof(config));
	config.saddr = (uint32_t)(uintptr_t)src;
	config.daddr = (uint32_t)(uintptr_t)g_dst;
	config.soff = 4;
	config.doff = 4;
	config.iter = 1;
	config.ssize = EDMA_32BIT;
	config.dsize = EDMA_32BIT;
	config.nbytes = PROBE_LEN;

	nxsem_init(&copy.done, 0, 0);
	copy.result = -ETIMEDOUT;

	int ret = imxrt_dmach_xfrsetup(ch, &config);

	if (ret == 0) {
		ret = imxrt_dmach_start(ch, copy_done, &copy);
	}

	if (ret == 0) {
		if (nxsem_tickwait(&copy.done, MSEC2TICK(100)) < 0) {
			imxrt_dmach_stop(ch);
			copy.result = -ETIMEDOUT;
		}

		ret = copy.result;
	}

	imxrt_dmach_free(ch);
	nxsem_destroy(&copy.done);
	up_invalidate_dcache((uintptr_t)g_dst, (uintptr_t)g_dst + sizeof(g_dst));
	return ret;
}

static bool all_zero(const uint8_t *buf, size_t len)
{
	uint8_t acc = 0;

	for (size_t i = 0; i < len; i++) {
		acc |= buf[i];
	}

	return acc == 0;
}

static int probe(void)
{
	printf("XRDC2 MCR 0x%08lx\n",
	       (unsigned long)getreg32(IMXRT_XRDC2_D0_BASE + IMXRT_XRDC2_MCR_OFFSET));
	printf("fenced region 0x%08lx-0x%08lx\n",
	       (unsigned long)(uintptr_t)_ssecmem, (unsigned long)(uintptr_t)_esecmem);

	for (size_t i = 0; i < sizeof(g_src); i++) {
		g_src[i] = 0xa5 ^ i;
	}

	up_clean_dcache((uintptr_t)g_src, (uintptr_t)g_src + sizeof(g_src));

	int ret = dma_copy(g_src);
	const bool control_ok = ret == 0 && memcmp(g_dst, g_src, PROBE_LEN) == 0;
	printf("eDMA from normal RAM: %s (%d)\n", control_ok ? "copied" : "FAILED", ret);

	up_clean_dcache((uintptr_t)_ssecmem, (uintptr_t)_ssecmem + PROBE_LEN);

	ret = dma_copy(_ssecmem);
	const bool refused = ret == -EIO && all_zero(g_dst, PROBE_LEN);
	explicit_bzero(g_dst, sizeof(g_dst));
	printf("eDMA from key region: %s (%d)\n", refused ? "refused" : "NOT REFUSED", ret);

	printf("%s\n", control_ok && refused ? "PASS" : "FAIL");
	return control_ok && refused ? 0 : 1;
}

static void usage(void)
{
	PRINT_MODULE_DESCRIPTION(
		"### Description\n"
		"Check the XRDC2 fence: one eDMA copy from normal RAM must land,\n"
		"one from the kernel key region must end in a bus error.\n"
	);
	PRINT_MODULE_USAGE_NAME_SIMPLE("xrdc2", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("probe", "Run both eDMA copies and report");
}

int xrdc2_main(int argc, char *argv[])
{
	if (argc == 2 && strcmp(argv[1], "probe") == 0) {
		return probe();
	}

	usage();
	return 1;
}
