/****************************************************************************
 *
 *   Copyright (C) 2026 Technology Innovation Institute. All rights reserved.
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
 * @file panic_flexspi_nor.cpp
 * Implementation of NXP IMX9 FlexSPI NOR panic mode reset for safe recovery
 * during kernel panic to prevent USB flashing mode entry.
 */

#include "panic_flexspi_nor.h"
#include <errno.h>
#include <nuttx/arch.h>

/* Register access macros */

#ifndef getreg32
static inline uint32_t getreg32(uintptr_t addr)
{
	return *(volatile uint32_t *)addr;
}
#endif

#ifndef putreg32
static inline void putreg32(uint32_t val, uintptr_t addr)
{
	*(volatile uint32_t *)addr = val;
}
#endif

/**
 * @brief Modify a 32-bit register with mask and set bits
 */
static inline void px4_imx9_flexspi_nor_modifyreg(uint32_t addr,
		uint32_t clearbits,
		uint32_t setbits)
{
	putreg32((getreg32(addr) & ~clearbits) | setbits, addr);
}

/* FlexSPI register offsets */

#define FLEXSPI_MCR0_OFFSET      0x00
#define FLEXSPI_IPCR0_OFFSET     0xa0
#define FLEXSPI_IPCR1_OFFSET     0xa4
#define FLEXSPI_IPCMD_OFFSET     0xb0
#define FLEXSPI_STS0_OFFSET      0xe0

/* MCR0 register bits */

#define FLEXSPI_MCR0_SWRESET     (1 << 0)

/* IPCR1 register bits - command sequence indices */

#define FLEXSPI_IPCR1_ISEQID_SHIFT   16
#define FLEXSPI_IPCR1_ISEQNUM_SHIFT  24

/* STS0 register bits */

#define FLEXSPI_STS0_SEQIDLE     (1 << 0)
#define FLEXSPI_STS0_ARBIDLE     (1 << 1)

/* IPCMD register bits */

#define FLEXSPI_IPCMD_TRG        (1 << 0)

/* Command sequence indices from imx9_flexspi_nor.c */

#define CMD_RESET_ENABLE         10  /* Index in LUT for RESET_ENABLE */
#define CMD_RESET_MEMORY         11  /* Index in LUT for RESET_MEMORY */

/* Timeout for busy wait loops (prevent infinite loops) */

#define FLEXSPI_PANIC_TIMEOUT    1000000

/**
 * @brief Wait for FlexSPI sequence to complete
 *
 * Busy-waits until the sequence completes or timeout occurs.
 * No OS services required.
 */
static int px4_imx9_flexspi_nor_wait_idle(uint32_t flexspi_base)
{
	uint32_t status_addr = flexspi_base + FLEXSPI_STS0_OFFSET;
	uint32_t timeout = FLEXSPI_PANIC_TIMEOUT;

	while ((getreg32(status_addr) & FLEXSPI_STS0_SEQIDLE) == 0 && timeout-- > 0) {
		/* Busy wait - no OS services */
	}

	if (timeout == 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * @brief Issue a command sequence to FlexSPI NOR device
 *
 * Configures and triggers an IP command on the NOR flash device.
 * This is a panic-safe operation using only direct register access.
 */
static int px4_imx9_flexspi_nor_issue_cmd(uint32_t flexspi_base,
		int port,
		int seq_index)
{
	uint32_t ipcr0_addr = flexspi_base + FLEXSPI_IPCR0_OFFSET;
	uint32_t ipcr1_addr = flexspi_base + FLEXSPI_IPCR1_OFFSET;
	uint32_t ipcmd_addr = flexspi_base + FLEXSPI_IPCMD_OFFSET;
	int ret;

	/* Set device address to 0 (not used for these commands) */

	putreg32(0, ipcr0_addr);

	/* Set data size to 0 and sequence parameters:
	 * - ISEQID: sequence index for the command
	 * - ISEQNUM: number of sequences (1 for simple commands)
	 * - IPAREN: no parity
	 */

	uint32_t ipcr1 = ((seq_index & 0xf) << FLEXSPI_IPCR1_ISEQID_SHIFT) |
					 ((1 & 0x7) << FLEXSPI_IPCR1_ISEQNUM_SHIFT);

	putreg32(ipcr1, ipcr1_addr);

	/* Trigger the IP command */

	putreg32(FLEXSPI_IPCMD_TRG, ipcmd_addr);

	/* Wait for sequence to complete */

	ret = px4_imx9_flexspi_nor_wait_idle(flexspi_base);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Busy-wait delay without OS services
 *
 * Simple microsecond delay using busy-wait loop.
 * Timing is approximate but sufficient for NOR reset requirements.
 */
static void px4_imx9_flexspi_nor_panic_delay_us(uint32_t microseconds)
{
	/* Rough estimate: ~1000 loops per microsecond on typical ARM Cortex-A
	 * This is conservative and will result in longer delays than requested
	 */

	volatile uint32_t loop_count = microseconds * 500;

	while (loop_count-- > 0) {
		/* Empty loop */
	}
}

/**
 * @brief Perform a panic-safe NOR flash reset
 *
 * Implementation of NOR reset without using OS services.
 * Safe to call from interrupt/panic context.
 */
int px4_imx9_flexspi_nor_panic_reset(uint32_t flexspi_base, int port)
{
	uint32_t mcr0_addr = flexspi_base + FLEXSPI_MCR0_OFFSET;
	int ret;

	/* Step 1: Send RESET_ENABLE command */

	ret = px4_imx9_flexspi_nor_issue_cmd(flexspi_base, port, CMD_RESET_ENABLE);

	if (ret < 0) {
		return ret;
	}

	/* Step 2: Small delay (1 microsecond) */

	px4_imx9_flexspi_nor_panic_delay_us(1);

	/* Step 3: Send RESET_MEMORY command */

	ret = px4_imx9_flexspi_nor_issue_cmd(flexspi_base, port, CMD_RESET_MEMORY);

	if (ret < 0) {
		return ret;
	}

	/* Step 4: Wait for bus to be idle (already done in issue_cmd) */

	/* Step 5: Perform FlexSPI software reset */

	px4_imx9_flexspi_nor_modifyreg(mcr0_addr, 0, FLEXSPI_MCR0_SWRESET);

	/* Small delay to allow reset to propagate */

	px4_imx9_flexspi_nor_panic_delay_us(10);

	return 0;
}
