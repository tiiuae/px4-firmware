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
#include <nuttx/atomic.h>
#include <nuttx/board.h>
#include <nuttx/sched.h>
#include "imx9_pmic.h"

#ifdef CONFIG_SMP
/* The CPU mask for all (valid) CPUs */

#define ALL_CPUS ((1 << CONFIG_SMP_NCPUS) - 1)

/* With SMP, we keep track on which CPUs have completed their reboot / shutdown */

static int g_cpus_ready;

/* Handle for nxsched_smp_call_async */

static struct smp_call_data_s g_reboot_data;
#endif

extern "C" void __start(void);
extern "C" int psci_cpu_off(void);

static void board_reset_enter_bootloader()
{
	/* WDOG_B pin is unused in current HW design.
	* Related configuration bits (WDOG_B_CFG) from PMIC RESET_CTRL
	* register are borrowed for stay in bootloader reason delivery.
	*
	* stay in bootloader: WDOG_B_CFG 11b
	*/

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(REBOOT_TO_BOOTLOADER);
#endif

	imx9_pmic_set_reset_ctrl(IMX9_PMIC_RESET_CTRL_DEFAULT |
				 IMX9_PMIC_RESET_CTRL_WDOG_COLD_RESET_MASK);

	/* Reset the whole SoC */

	imx9_pmic_reset();

	while (1);
}

static void board_reset_enter_bootloader_and_continue_boot()
{
	/* WDOG_B pin is unused in current HW design.
	* Related configuration bits (WDOG_B_CFG) from PMIC RESET_CTRL
	* register are borrowed for stay in bootloader reason delivery.
	*
	* With PMIC register power on value system will boot normally.
	*/

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(REBOOT_TO_BOOTLOADER_CONTINUE);
#endif

	imx9_pmic_set_reset_ctrl(IMX9_PMIC_RESET_CTRL_DEFAULT);

	/* Reset the whole SoC */

	imx9_pmic_reset();

	while (1);
}

/* This is called for each CPU when doing warm reboot */

static int board_reset_enter_app(FAR void *arg)
{
#ifdef CONFIG_SMP
	/* Notify that this CPU is ready */

	atomic_fetch_add(&g_cpus_ready, 1);

	/* Wait for other CPUs to shut down */

	while (atomic_load(&g_cpus_ready) < CONFIG_SMP_NCPUS);

	if (this_cpu() != 0) {
		/* Make sure caches are written to ram before shutting off */

		up_clean_dcache_all();

		/* The secondary CPU needs to be turned off */

		psci_cpu_off();
	}

#endif

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(REBOOT_REQUEST);
#endif

	/* The primary CPU jumps to start */

	__start();

	/* Never reached */

	return 0;
}

int board_reset(int status)
{
	DEBUGASSERT(!up_interrupt_context());

	/* First check if doing a PMIC reset */

	if (status == REBOOT_TO_BOOTLOADER) {
		/* PMIC reset, stay in bootloader */

		board_reset_enter_bootloader();

	} else if (status == REBOOT_TO_BOOTLOADER_CONTINUE) {
		/* PMIC reset, reboot */

		board_reset_enter_bootloader_and_continue_boot();
	}

	/* If we and up here, continue with warm boot */

	/* Disable local interrupts */

	up_irq_save();

	/* Don't ever schedule away any more on this CPU */

	sched_lock();

#ifdef CONFIG_SMP

	/* Now that the CPU cannot change, start the reboot process.
	 * All the other CPUs will end up in interrupt handler in
	 * board_reset_enter_app
	 */

	g_reboot_data.func = board_reset_enter_app;
	g_reboot_data.arg  = NULL;
	g_cpus_ready       = 0;

	/* Reboot or shut down the other CPUs via SMP call */

	cpu_set_t cpuset = ALL_CPUS;
	CPU_CLR(this_cpu(), &cpuset);
	nxsched_smp_call_async(cpuset, &g_reboot_data);
#endif

	/* Enter reboot function */

	board_reset_enter_app(NULL);

	return 0;
}
