/****************************************************************************
 * boards/risc-v/mpfs/icicle/src/mpfs_pwm.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <debug.h>
#include <string.h>
#include <limits.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>
#include "mpfs_corepwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_pwm_setup
 *
 * Description:
 *
 *   Initialize PWM and register PWM devices
 *
 ****************************************************************************/

#define PWM_DEV_NAME "/dev/pwm%d"

int mpfs_pwm_setup(void)
{
	int npwm = 0;
	char devname[sizeof(PWM_DEV_NAME)];        /* Buffer for device name    */
	struct pwm_lowerhalf_s *lower_half = NULL; /* lower-half handle         */
	int config_npwm = 0;                       /* Number of channels in use */

	/* The underlying CorePWM driver "knows" there are up to 16 channels
	 * available for each timer device, so we don't have to do anything
	 * special here.
	 */

#ifdef CONFIG_MPFS_COREPWM0
	config_npwm++;
#endif
#ifdef CONFIG_MPFS_COREPWM1
	config_npwm++;
#endif

	for (npwm = 0; npwm < config_npwm; npwm++) {
		lower_half = mpfs_corepwm_init(npwm);

		/* If we can't get the lower-half handle, skip and keep going. */

		if (lower_half) {
			/* Translate the peripheral number to a device name. */
			snprintf(devname, sizeof(devname), PWM_DEV_NAME, npwm);
			pwm_register(devname, lower_half);
		}
	}

	return 0;
}
