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
 * @file parameters_ioctl.cpp
 *
 * Protected build kernel space interface to global parameter store.
 */

#define PARAM_IMPLEMENTATION

#include <errno.h>

#include "param.h"
#include "parameters_ioctl.h"
#include <px4_platform_common/defines.h>
#include <px4_platform/board_ctrl.h>

static constexpr int PARAM_GROUP_MAX = 256;

static int param_reset_group(const paramiocresetgroup_t &data)
{
	if (data.type != PARAM_RESET_EXCLUDES && data.type != PARAM_RESET_SPECIFIC) {
		param_reset_all();
		return OK;
	}

	const int n = data.num_in_group;

	if (n < 0 || n > PARAM_GROUP_MAX || !px4_user_ok(data.group, n * sizeof(*data.group))) {
		return -EFAULT;
	}

	const char **group = new const char *[n > 0 ? n : 1];

	if (group == nullptr) {
		return -ENOMEM;
	}

	for (int i = 0; i < n; i++) {
		group[i] = data.group[i];

		if (group[i] == nullptr || !px4_user_ok(group[i], 1)) {
			delete[] group;
			return -EFAULT;
		}
	}

	if (data.type == PARAM_RESET_EXCLUDES) {
		param_reset_excludes(group, n);

	} else {
		param_reset_specific(group, n);
	}

	delete[] group;
	return OK;
}

int	param_ioctl(unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case PARAMIOCNOTIFY:
		param_notify_changes();
		return OK;

	case PARAMIOCFIND: {
			px4_user_arg<paramiocfind_t> d;

			if (!d.in(arg) || d->name == nullptr || !px4_user_ok(d->name, 1)) {
				return -EFAULT;
			}

			((paramiocfind_t *)arg)->ret = d->notification ? param_find(d->name) : param_find_no_notification(d->name);
			return OK;
		}

	case PARAMIOCCOUNTUSED: {
			px4_user_arg<paramioccountused_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramioccountused_t *)arg)->ret = param_count_used();
			return OK;
		}

	case PARAMIOCFORUSEDINDEX: {
			px4_user_arg<paramiocforusedindex_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocforusedindex_t *)arg)->ret = param_for_used_index(d->index);
			return OK;
		}

	case PARAMIOCGETUSEDINDEX: {
			px4_user_arg<paramiocgetusedindex_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocgetusedindex_t *)arg)->ret = param_get_used_index(d->param);
			return OK;
		}

	case PARAMIOCUNSAVED: {
			px4_user_arg<paramiocunsaved_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocunsaved_t *)arg)->ret = param_value_unsaved(d->param);
			return OK;
		}

	case PARAMIOCGET: {
			px4_user_arg<paramiocget_t> d;

			if (!d.in(arg) || !px4_user_ok(d->val, sizeof(int32_t))) {
				return -EFAULT;
			}

			((paramiocget_t *)arg)->ret = d->deflt ? param_get_default_value(d->param, d->val) : param_get(d->param, d->val);
			return OK;
		}

	case PARAMIOCAUTOSAVE: {
			px4_user_arg<paramiocautosave_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			param_control_autosave(d->enable);
			return OK;
		}

	case PARAMIOCSET: {
			px4_user_arg<paramiocset_t> d;

			if (!d.in(arg) || !px4_user_ok(d->val, sizeof(int32_t))) {
				return -EFAULT;
			}

			((paramiocset_t *)arg)->ret = d->notification ? param_set(d->param, d->val) : param_set_no_notification(d->param,
						      d->val);
			return OK;
		}

	case PARAMIOCUSED: {
			px4_user_arg<paramiocused_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocused_t *)arg)->ret = param_used(d->param);
			return OK;
		}

	case PARAMIOCSETUSED: {
			px4_user_arg<paramiocsetused_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			param_set_used(d->param);
			return OK;
		}

	case PARAMIOCSETDEFAULT: {
			px4_user_arg<paramiocsetdefault_t> d;

			if (!d.in(arg) || !px4_user_ok(d->val, sizeof(int32_t))) {
				return -EFAULT;
			}

			((paramiocsetdefault_t *)arg)->ret = param_set_default_value(d->param, d->val);
			return OK;
		}

	case PARAMIOCRESET: {
			px4_user_arg<paramiocreset_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocreset_t *)arg)->ret = d->notification ? param_reset(d->param) : param_reset_no_notification(d->param);
			return OK;
		}

	case PARAMIOCRESETGROUP: {
			px4_user_arg<paramiocresetgroup_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			return param_reset_group(*d);
		}

	case PARAMIOCSAVEDEFAULT: {
			px4_user_arg<paramiocsavedefault_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocsavedefault_t *)arg)->ret = param_save_default(d->blocking);
			return OK;
		}

	case PARAMIOCLOADDEFAULT: {
			px4_user_arg<paramiocloaddefault_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiocloaddefault_t *)arg)->ret = param_load_default();
			return OK;
		}

	case PARAMIOCEXPORT: {
			px4_user_arg<paramiocexport_t> d;

			if (!d.in(arg) || (d->filename != nullptr && !px4_user_ok(d->filename, 1))) {
				return -EFAULT;
			}

			((paramiocexport_t *)arg)->ret = param_export(d->filename, nullptr);
			return OK;
		}

	case PARAMIOCHASH: {
			px4_user_arg<paramiochash_t> d;

			if (!d.in(arg)) {
				return -EFAULT;
			}

			((paramiochash_t *)arg)->ret = param_hash_check();
			return OK;
		}

	default:
		return -ENOTTY;
	}
}
