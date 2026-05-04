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
 * @file panic_lpi2c.cpp
 * Implementation of NXP IMX9 LPI2C panic mode operations for kernel panic
 * recovery scenarios.
 */

#include "panic_lpi2c.h"

extern "C"
{
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"
}

#include "hardware/imx9_ccm.h"
#include "hardware/imx9_lpi2c.h"

#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#define IMX9_LPI2C_PANIC_TIMEOUT  1000000
#define PMIC_I2C_FREQ_HZ  400000

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
static inline void px4_imx9_lpi2c_raw_modifyreg(uint32_t addr,
		uint32_t clearbits,
		uint32_t setbits)
{
	putreg32((getreg32(addr) & ~clearbits) | setbits, addr);
}

/**
 * @brief Configure LPI2C controller based on port number
 */
int px4_imx9_lpi2c_raw_config(int port, struct px4_imx9_lpi2c_panic_ctx_s *ctx)
{
	switch (port) {
#ifdef CONFIG_IMX9_LPI2C1

	case 1:
		ctx->base = IMX9_LPI2C1_BASE;
		ctx->clk_root = CCM_CR_LPI2C1;
		ctx->clk_gate = CCM_LPCG_LPI2C1;
		ctx->busy_idle = CONFIG_IMX9_LPI2C1_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C1_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C1_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C2

	case 2:
		ctx->base = IMX9_LPI2C2_BASE;
		ctx->clk_root = CCM_CR_LPI2C2;
		ctx->clk_gate = CCM_LPCG_LPI2C2;
		ctx->busy_idle = CONFIG_IMX9_LPI2C2_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C2_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C2_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C3

	case 3:
		ctx->base = IMX9_LPI2C3_BASE;
		ctx->clk_root = CCM_CR_LPI2C3;
		ctx->clk_gate = CCM_LPCG_LPI2C3;
		ctx->busy_idle = CONFIG_IMX9_LPI2C3_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C3_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C3_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C4

	case 4:
		ctx->base = IMX9_LPI2C4_BASE;
		ctx->clk_root = CCM_CR_LPI2C4;
		ctx->clk_gate = CCM_LPCG_LPI2C4;
		ctx->busy_idle = CONFIG_IMX9_LPI2C4_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C4_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C4_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C5

	case 5:
		ctx->base = IMX9_LPI2C5_BASE;
		ctx->clk_root = CCM_CR_LPI2C5;
		ctx->clk_gate = CCM_LPCG_LPI2C5;
		ctx->busy_idle = CONFIG_IMX9_LPI2C5_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C5_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C5_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C6

	case 6:
		ctx->base = IMX9_LPI2C6_BASE;
		ctx->clk_root = CCM_CR_LPI2C6;
		ctx->clk_gate = CCM_LPCG_LPI2C6;
		ctx->busy_idle = CONFIG_IMX9_LPI2C6_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C6_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C6_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C7

	case 7:
		ctx->base = IMX9_LPI2C7_BASE;
		ctx->clk_root = CCM_CR_LPI2C7;
		ctx->clk_gate = CCM_LPCG_LPI2C7;
		ctx->busy_idle = CONFIG_IMX9_LPI2C7_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C7_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C7_FILTSDA;
		return OK;
#endif
#ifdef CONFIG_IMX9_LPI2C8

	case 8:
		ctx->base = IMX9_LPI2C8_BASE;
		ctx->clk_root = CCM_CR_LPI2C8;
		ctx->clk_gate = CCM_LPCG_LPI2C8;
		ctx->busy_idle = CONFIG_IMX9_LPI2C8_BUSYIDLE;
		ctx->filtscl = CONFIG_IMX9_LPI2C8_FILTSCL;
		ctx->filtsda = CONFIG_IMX9_LPI2C8_FILTSDA;
		return OK;
#endif

	default:
		return -ENODEV;
	}
}

/****************************************************************************
 * Name: px4_imx9_lpi2c_raw_setclock
 *
 * Description:
 *   Set the I2C clock
 *   This function is partly a copy of NuttX arch imx9_lpi2c_setclock
 *
 ****************************************************************************/

static int px4_imx9_lpi2c_raw_setclock(struct px4_imx9_lpi2c_panic_ctx_s *ctx,
				       uint32_t frequency)
{
	uint32_t src_freq = 0;
	uint32_t regval;
	uint32_t men;
	uint32_t prescale;
	uint32_t best_prescale = 0;
	uint32_t best_clk_hi = 0;
	uint32_t abs_error = 0;
	uint32_t best_error = UINT32_MAX;
	uint32_t clk_hi_cycle;
	uint32_t computed_rate;
	uint32_t count;
	int ret;

	ret = imx9_get_rootclock(ctx->clk_root, &src_freq);

	if (ret < 0 || src_freq == 0) {
		return -ENODEV;
	}

	/* Disable the selected LPI2C peripheral to configure the new clock. */

	men = getreg32(ctx->base + IMX9_LPI2C_MCR_OFFSET) & LPI2C_MCR_MEN;

	if (men != 0) {
		px4_imx9_lpi2c_raw_modifyreg(ctx->base + IMX9_LPI2C_MCR_OFFSET,
					     LPI2C_MCR_MEN, 0);
	}

	/* LPI2C output frequency = (Source Clock (Hz)/ 2^prescale) /
	 *   (CLKLO + 1 + CLKHI + 1 + ALIGN_DOWN((2 + FILTSCL)/2^prescale)
	 *
	 * Assume  CLKLO = 2 * CLKHI, SETHOLD = CLKHI, DATAVD = CLKHI / 2
	 */

	for (prescale = 1; (prescale <= 128) && (best_error != 0); prescale *= 2) {
		for (clk_hi_cycle = 1; clk_hi_cycle < 32; clk_hi_cycle++) {
			if (clk_hi_cycle == 1) {
				computed_rate = (src_freq / prescale) / (6 + (2 / prescale));

			} else {
				computed_rate = (src_freq / prescale) /
						((3 * clk_hi_cycle + 2) + (2 / prescale));
			}

			if (frequency > computed_rate) {
				abs_error = frequency - computed_rate;

			} else {
				abs_error = computed_rate - frequency;
			}

			if (abs_error < best_error) {
				best_prescale = prescale;
				best_clk_hi = clk_hi_cycle;
				best_error = abs_error;

				if (abs_error == 0) {
					break;
				}
			}
		}
	}

	regval = LPI2C_MCCR0_CLKHI(best_clk_hi);

	if (best_clk_hi < 2) {
		regval |= LPI2C_MCCR0_CLKLO(3) | LPI2C_MCCR0_SETHOLD(2) |
			  LPI2C_MCCR0_DATAVD(1);

	} else {
		regval |= LPI2C_MCCR0_CLKLO(2 * best_clk_hi) |
			  LPI2C_MCCR0_SETHOLD(best_clk_hi) |
			  LPI2C_MCCR0_DATAVD(best_clk_hi / 2);
	}

	putreg32(regval, ctx->base + IMX9_LPI2C_MCCR0_OFFSET);

	for (count = 0; count < 8; count++) {
		if (best_prescale == (1u << count)) {
			best_prescale = count;
			break;
		}
	}

	px4_imx9_lpi2c_raw_modifyreg(ctx->base + IMX9_LPI2C_MCFGR1_OFFSET,
				     LPI2C_MCFGR1_PRESCALE_MASK,
				     LPI2C_MCFGR1_PRESCALE(best_prescale));

	/* Re-enable LPI2C */

	if (men != 0) {
		px4_imx9_lpi2c_raw_modifyreg(ctx->base + IMX9_LPI2C_MCR_OFFSET,
					     0, LPI2C_MCR_MEN);
	}

	return OK;
}

/**
 * @brief Wait for LPI2C status conditions
 */
static int px4_imx9_lpi2c_raw_wait_status(uint32_t base, uint32_t set_mask,
		uint32_t clear_mask)
{
	uint32_t status;

	for (int timeout = IMX9_LPI2C_PANIC_TIMEOUT; timeout > 0; timeout--) {
		status = getreg32(base + IMX9_LPI2C_MSR_OFFSET);

		if ((status & LPI2C_MSR_ERROR_MASK) != 0) {
			putreg32(status & LPI2C_MSR_ERROR_MASK, base + IMX9_LPI2C_MSR_OFFSET);
			return -EIO;
		}

		if ((status & set_mask) == set_mask && (status & clear_mask) == 0) {
			return OK;
		}
	}

	return -ETIMEDOUT;
}

/**
 * @brief Prepare LPI2C controller for operation
 */
int px4_imx9_lpi2c_raw_prepare(int port, struct px4_imx9_lpi2c_panic_ctx_s *ctx)
{
	int ret;

	ret = px4_imx9_lpi2c_raw_config(port, ctx);

	if (ret < 0) {
		return ret;
	}

	imx9_ccm_gate_on(ctx->clk_gate, true);

	putreg32(0, ctx->base + IMX9_LPI2C_MIER_OFFSET);
	putreg32(0, ctx->base + IMX9_LPI2C_MDER_OFFSET);
	putreg32(LPI2C_MCR_RST, ctx->base + IMX9_LPI2C_MCR_OFFSET);
	putreg32(0, ctx->base + IMX9_LPI2C_MCR_OFFSET);
	putreg32(LPI2C_MCR_DOZEN | LPI2C_MCR_RTF | LPI2C_MCR_RRF,
		 ctx->base + IMX9_LPI2C_MCR_OFFSET);
	putreg32(LPI2C_MCR_DOZEN, ctx->base + IMX9_LPI2C_MCR_OFFSET);

	px4_imx9_lpi2c_raw_modifyreg(ctx->base + IMX9_LPI2C_MCFGR0_OFFSET,
				     LPI2C_MCFG0_HREN | LPI2C_MCFG0_HRSEL,
				     LPI2C_MCFG0_HRPOL);
	px4_imx9_lpi2c_raw_modifyreg(ctx->base + IMX9_LPI2C_MCFGR1_OFFSET,
				     LPI2C_MCFGR1_IGNACK | LPI2C_MCFGR1_AUTOSTOP, 0);
	putreg32(LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(0),
		 ctx->base + IMX9_LPI2C_MFCR_OFFSET);
	putreg32(LPI2C_MCFG2_BUSIDLE(ctx->busy_idle) |
		 LPI2C_MCFG2_FILTSCL_CYCLES(ctx->filtscl) |
		 LPI2C_MCFG2_FILTSDA_CYCLES(ctx->filtsda),
		 ctx->base + IMX9_LPI2C_MCFGR2_OFFSET);
	putreg32(LPI2C_MCFG3_PINLOW_CYCLES(0), ctx->base + IMX9_LPI2C_MCFGR3_OFFSET);

	ret = px4_imx9_lpi2c_raw_setclock(ctx, PMIC_I2C_FREQ_HZ);

	if (ret < 0) {
		return ret;
	}

	putreg32(0xffffffff, ctx->base + IMX9_LPI2C_MSR_OFFSET);
	px4_imx9_lpi2c_raw_modifyreg(ctx->base + IMX9_LPI2C_MCR_OFFSET,
				     0, LPI2C_MCR_MEN);

	return OK;
}

/**
 * @brief Perform I2C transfer
 */
int px4_imx9_lpi2c_raw_transfer(struct px4_imx9_lpi2c_panic_ctx_s *ctx, uint8_t addr,
				const uint8_t *buffer, int buflen)
{
	int ret;

	ret = px4_imx9_lpi2c_raw_wait_status(ctx->base, LPI2C_MSR_TDF, 0);

	if (ret < 0) {
		return ret;
	}

	putreg32(LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA(I2C_WRITEADDR8(addr)),
		 ctx->base + IMX9_LPI2C_MTDR_OFFSET);

	for (int i = 0; i < buflen; i++) {
		ret = px4_imx9_lpi2c_raw_wait_status(ctx->base, LPI2C_MSR_TDF, 0);

		if (ret < 0) {
			return ret;
		}

		putreg32(LPI2C_MTDR_CMD_TXD | LPI2C_MTDR_DATA(buffer[i]),
			 ctx->base + IMX9_LPI2C_MTDR_OFFSET);
	}

	ret = px4_imx9_lpi2c_raw_wait_status(ctx->base, LPI2C_MSR_TDF, 0);

	if (ret < 0) {
		return ret;
	}

	putreg32(LPI2C_MTDR_CMD_STOP, ctx->base + IMX9_LPI2C_MTDR_OFFSET);

	ret = px4_imx9_lpi2c_raw_wait_status(ctx->base, LPI2C_MSR_SDF, LPI2C_MSR_MBF);

	if (ret < 0) {
		return ret;
	}

	putreg32(LPI2C_MSR_SDF | LPI2C_MSR_EPF | LPI2C_MSR_ERROR_MASK,
		 ctx->base + IMX9_LPI2C_MSR_OFFSET);
	return OK;
}

/**
 * @brief Abort pending I2C transfer
 */
void px4_imx9_lpi2c_raw_abort(struct px4_imx9_lpi2c_panic_ctx_s *ctx)
{
	putreg32(LPI2C_MTDR_CMD_STOP, ctx->base + IMX9_LPI2C_MTDR_OFFSET);
	putreg32(LPI2C_MCR_DOZEN | LPI2C_MCR_RTF | LPI2C_MCR_RRF |
		 LPI2C_MCR_MEN,
		 ctx->base + IMX9_LPI2C_MCR_OFFSET);
	putreg32(0xffffffff, ctx->base + IMX9_LPI2C_MSR_OFFSET);
}
