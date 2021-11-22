/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * @file init.c
 *
 * Saluki-specific early startup code.  This file implements the
 * board_app_initializ() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <sys/mount.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>
#include <chip.h>
#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_dma_alloc.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS


/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	syslog(LOG_DEBUG, "board_peripheral_reset\n");
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{
	syslog(LOG_DEBUG, "board_on_reset\n");
}

/************************************************************************************
 * Name: mpfs_boardinitialize
 *
 * Description:
 *   This entry point is called early in the initialization -- after all memory
 *  has been configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
mpfs_boardinitialize(void)
{
	syslog(LOG_DEBUG, "mpfs_boardinitialize\n");
	board_autoled_initialize();

	/* this call exists to fix a weird linking issue */
	up_udelay(0);

	/* Configure Safety button GPIO */
	mpfs_configgpio(GPIO_BTN_SAFETY);

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/


__EXPORT int board_app_initialize(uintptr_t arg)
{
	int ret = OK;

	_alert("saluki: board_app_initialize\n");

	/* Power on Interfaces */

	/* configure SPI interfaces and devices (after we determined the HW version) */
#ifdef CONFIG_SPI
	board_spidev_initialize();
	board_spibus_initialize();
#endif

#ifdef CONFIG_MMCSD
	ret = mpfs_board_emmcsd_init();

	if (ret != OK) {
		led_on(LED_RED);
		syslog(LOG_ERR, "ERROR: Failed to initialize SD card");
	}

#endif /* CONFIG_MMCSD */

#ifdef CONFIG_MPFS_HAVE_COREPWM
	/* Configure PWM peripheral interfaces */

	ret = mpfs_pwm_setup();

	if (ret < 0) {
		syslog(LOG_ERR, "Failed to initialize CorePWM driver: %d\n", ret);
	}

#endif

#ifdef CONFIG_FS_PROCFS
	/* Mount the procfs file system */

	ret = mount(NULL, "/proc", "procfs", 0, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
	}

#endif

#ifdef CONFIG_MTD_M25P

	ret = mpfs_board_spinor_init(mpfs_spibus_initialize(1));

	if (ret < 0) {
		return ret;
	}

	/* Mount LFS on the block1 */
	ret = nx_mount("/dev/mtdblock1", "/fs/lfs", "littlefs", 0, "autoformat");

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to mount LFS filesystem\n");
		return ret;
	}

#endif

	return ret;
}
