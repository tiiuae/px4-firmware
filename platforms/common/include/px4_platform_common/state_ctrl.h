/****************************************************************************
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Technology Innovation Institute
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file state_ctrl.h
 */

#ifndef STATE_CTRL_H
#define STATE_CTRL_H

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT)

#include <px4_platform/board_ctrl.h>
#include <px4_platform_common/defines.h>

#define _STATECTRLIOC(_n) (_PX4_IOC(_STATECTRLIOCBASE, _n))

#define STATECTRLIOCREADMOISTATE    _STATECTRLIOC(1)
typedef struct statectrliocreadmoistate {
	uint8_t *state;
	int ret;
} statectrliocreadmoistate_t;

#define STATECTRLIOCWRITEMOISTATE   _STATECTRLIOC(2)
typedef struct statectrliocwritemoistate {
	uint8_t state;
	int ret;
} statectrliocwritemoistate_t;

#define STATECTRLIOCREADFWUPD       _STATECTRLIOC(3)
typedef struct statectrliocreadfwupd {
	uint8_t *fwupd;
	int ret;
} statectrliocreadfwupd_t;

#define STATECTRLIOCWRITEFWUPD      _STATECTRLIOC(4)
typedef struct statectrliocwritefwupd {
	uint8_t *fwupd;
	int ret;
} statectrliocwritefwupd_t;

#endif // __PX4_NUTTX && !CONFIG_BUILD_FLAT

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT)
int statectrl_ioctl(unsigned int cmd, unsigned long arg);
#endif // __PX4_NUTTX && !CONFIG_BUILD_FLAT

int statectrl_get_moi_state(uint8_t *state);
int statectrl_set_moi_state(uint8_t state);
int statectrl_get_fwupd(uint8_t *fwupd);
int statectrl_set_fwupd(uint8_t *fwupd);

void statectrl_init(void);

#ifdef __cplusplus
}
#endif


#endif // STATE_CTRL_H
