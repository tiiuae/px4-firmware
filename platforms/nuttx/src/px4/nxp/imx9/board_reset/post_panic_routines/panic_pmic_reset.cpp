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
 * @file panic_pmic_reset.cpp
 * Implementation of NXP imx9 based PMIC reset functionality for kernel panic
 * recovery.
 */

#include "panic_lpi2c.h"
#include "imx9_pmic.h"

/* PCA9451A PMIC register map used by panic reset path. */

#define PCA9451A_I2C_ADDR 0x25
#define REG_SW_RST        0x06
#define REG_RESET_CTRL    0x08
#define COLD_RESET        0x64

/**
 * @brief Write a register to the PCA9451A PMIC via I2C
 */
static int px4_imx9_pmic_panic_write_reg(struct px4_imx9_lpi2c_panic_ctx_s *ctx, uint8_t reg,
		uint8_t val)
{
	uint8_t buffer[2] = {reg, val};

	return px4_imx9_lpi2c_raw_transfer(ctx, PCA9451A_I2C_ADDR, buffer, 2);
}

/****************************************************************************
 * Name: px4_imx9_pmic_panic_reset_with_ctrl
 *
 * Description:
 *   Program PMIC reset control and trigger reset with a single panic I2C
 *   controller setup.
 *
 ****************************************************************************/

int px4_imx9_pmic_panic_reset_with_ctrl(uint8_t reset_ctrl)
{
	struct px4_imx9_lpi2c_panic_ctx_s ctx;
	int ret;

	// Initialize lpi2c bus for transfer
	ret = px4_imx9_lpi2c_raw_prepare(CONFIG_IMX9_PMIC_I2C, &ctx);

	if (ret < 0) {
		return ret;
	}

	// Write reset control register to configure reset behavior
	ret = px4_imx9_pmic_panic_write_reg(&ctx, REG_RESET_CTRL, reset_ctrl);

	if (ret < 0) {
		px4_imx9_lpi2c_raw_abort(&ctx);
		return ret;
	}

	// Trigger cold reset by writing to software reset register
	ret = px4_imx9_pmic_panic_write_reg(&ctx, REG_SW_RST, COLD_RESET);

	if (ret < 0) {
		px4_imx9_lpi2c_raw_abort(&ctx);
	}

	return ret;
}
