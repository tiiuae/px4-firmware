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
 * @file panic_flexspi_nor.h
 * IMX9 FlexSPI NOR panic mode reset operations for safe recovery during kernel panic.
 */

#ifndef __IMX9_FLEXSPI_NOR_PANIC_H__
#define __IMX9_FLEXSPI_NOR_PANIC_H__

#include <stdint.h>

/**
 * @brief Perform a panic-safe NOR flash reset
 *
 * This function resets the NOR flash memory using direct hardware register access
 * without relying on OS services. This is essential for panic recovery to prevent
 * the NXP ROM from entering USB flashing mode.
 *
 * The operation sequence:
 * - Sends RESET_ENABLE command to NOR flash
 * - Sends RESET_MEMORY command to perform the actual reset
 * - Waits for device ready status
 * - Performs FlexSPI controller software reset
 *
 * @param flexspi_base Base address of FlexSPI controller
 * @param port FlexSPI port number (typically 0 for primary NOR device)
 * @return 0 on success, negative errno on failure
 *
 * @note This function is designed for use during kernel panic when OS services
 *       may be unavailable. It uses only direct hardware register access.
 */
int px4_imx9_flexspi_nor_panic_reset(uint32_t flexspi_base, int port);

#endif /* __IMX9_FLEXSPI_NOR_PANIC_H__ */
