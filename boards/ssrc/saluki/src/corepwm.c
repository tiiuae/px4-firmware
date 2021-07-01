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
 * @file corepwm.c
 *
 * Saluki-specific CorePWM inititalization.
 *
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

#include <nuttx/timers/pwm.h>
#include "mpfs_corepwm.h"


/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Name: mpfs_corepwm_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Returned Value:
 *   Zero (OK) is returned on always.
 *
 ****************************************************************************/

int mpfs_corepwm_initialize(void)
{
#ifdef CONFIG_MPFS_HAVE_COREPWM

	/* CorePWM blocks are behind FIC3, enable it */

	SYSREG->SUBBLK_CLOCK_CR |= (SUBBLK_CLOCK_CR_FIC3_MASK);
	SYSREG->SOFT_RESET_CR   &= (uint32_t)~(SOFT_RESET_CR_FIC3_MASK | SOFT_RESET_CR_FPGA_MASK);

	/* Configure PWM peripheral interfaces */

	int npwm = 0;                              /* hardware device enumerator     */
	char devname[20];                          /* buffer for the PWM device name */
	struct pwm_lowerhalf_s *lower_half = NULL; /* lower-half driver handle       */

	/* The underlying CorePWM driver "knows" there are up to 16 channels
	* available for each timer device, so we don't have to do anything
	* special here.
	*/
	int config_npwm = 0;

	#ifdef CONFIG_MPFS_COREPWM0
	config_npwm++;
	#endif

	#ifdef CONFIG_MPFS_COREPWM1
	config_npwm++;
	#endif

	for (npwm = 0; npwm < config_npwm; npwm++)
	{
		lower_half = mpfs_corepwminitialize(npwm);

		/* If we can't get the lower-half handle, skip and keep going. */

		if (lower_half)
		{	/* Translate the peripheral number to a device name. */
			snprintf(devname, sizeof(devname), "/dev/corepwm%d", npwm);
			pwm_register(devname, lower_half);
		}
	}
	/* Configure the HW based on the manifest */
#endif

	return OK;
}
