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
 * @file panic_lpi2c.h
 * IMX9 LPI2C panic mode operations for accessing devices during kernel panic.
 */

#ifndef __IMX9_LPI2C_PANIC_H__
#define __IMX9_LPI2C_PANIC_H__

#include <stdint.h>

/**
 * @brief IMX9 LPI2C panic context structure
 *
 * Contains configuration and state for LPI2C operation in panic mode.
 */
struct px4_imx9_lpi2c_panic_ctx_s {
	uint32_t base;       /**< LPI2C base address */
	int clk_root;        /**< Clock root configuration */
	int clk_gate;        /**< Clock gate configuration */
	uint16_t busy_idle;  /**< Busy idle timing */
	uint8_t filtscl;     /**< SCL filter cycles */
	uint8_t filtsda;     /**< SDA filter cycles */
};

/**
 * @brief Configure LPI2C controller in panic mode
 *
 * @param port LPI2C port number (1-8)
 * @param ctx Panic context structure to populate
 * @return OK on success, negative errno on failure
 */
int px4_imx9_lpi2c_raw_config(int port, struct px4_imx9_lpi2c_panic_ctx_s *ctx);

/**
 * @brief Prepare LPI2C controller for panic mode operation
 *
 * Initializes and configures the specified LPI2C controller with panic-safe
 * settings and sets up the bus for communication.
 *
 * @param port LPI2C port number (1-8)
 * @param ctx Panic context structure to populate
 * @return OK on success, negative errno on failure
 */
int px4_imx9_lpi2c_raw_prepare(int port, struct px4_imx9_lpi2c_panic_ctx_s *ctx);

/**
 * @brief Perform I2C transfer in panic mode
 *
 * Sends a START condition, address, data bytes, and STOP condition
 * without interrupts or blocking operations.
 *
 * @param ctx Panic context structure
 * @param addr I2C device address
 * @param buffer Data buffer to transmit
 * @param buflen Number of bytes to transmit
 * @return OK on success, negative errno on failure
 */
int px4_imx9_lpi2c_raw_transfer(struct px4_imx9_lpi2c_panic_ctx_s *ctx, uint8_t addr,
				const uint8_t *buffer, int buflen);

/**
 * @brief Abort pending I2C transfer
 *
 * @param ctx Panic context structure
 */
void px4_imx9_lpi2c_raw_abort(struct px4_imx9_lpi2c_panic_ctx_s *ctx);

#endif /* __IMX9_LPI2C_PANIC_H__ */
