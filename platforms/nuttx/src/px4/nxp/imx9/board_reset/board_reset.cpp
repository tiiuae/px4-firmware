/****************************************************************************
 *
 *   Copyright (C) 2024 Technology Innovation Institute. All rights reserved.
 *   Author: @author Jukka Laitinen <jukkax@ssrc.tii.ae>
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
 * @file board_reset.cpp
 * Implementation of Microchip PolarFire based Board RESET API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/shutdown.h>
#include <errno.h>
#include <nuttx/board.h>
#include <debug.h>
#include "imx9_pmic.h"

extern "C" void __start(void);

static void board_reset_enter_bootloader()
{
	/* WDOG_B pin is unused in current HW design.
	* Related configuration bits (WDOG_B_CFG) from PMIC RESET_CTRL
	* register are borrowed for stay in bootloader reason delivery.
	*
	* stay in bootloader: WDOG_B_CFG 11b
	*/

	imx9_pmic_set_reset_ctrl(IMX9_PMIC_RESET_CTRL_DEFAULT |
				 IMX9_PMIC_RESET_CTRL_WDOG_COLD_RESET_MASK);

	/* Reset the whole SoC */

	up_systemreset();
}

static void board_reset_enter_bootloader_and_continue_boot()
{
	/* WDOG_B pin is unused in current HW design.
	* Related configuration bits (WDOG_B_CFG) from PMIC RESET_CTRL
	* register are borrowed for stay in bootloader reason delivery.
	*
	* With PMIC register power on value system will boot normally.
	*/

	imx9_pmic_set_reset_ctrl(IMX9_PMIC_RESET_CTRL_DEFAULT);

	/* Reset the whole SoC */

	up_systemreset();
}

static void board_reset_enter_app()
{
	__start();
}

int board_reset(int status)
{
#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

	if (status == REBOOT_TO_BOOTLOADER) {
		board_reset_enter_bootloader();

	} else if (status == REBOOT_TO_BOOTLOADER_CONTINUE) {
		board_reset_enter_bootloader_and_continue_boot();
	}

	/* Just reboot via reset vector */

	board_reset_enter_app();

	return 0;
}
