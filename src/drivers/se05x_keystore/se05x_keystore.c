/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
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

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>

#include <nuttx/crypto/se05x.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_mtd.h>
#include "keystore_backend_definitions.h"

#define SE05X_KEYSTORE_ID(idx) (0x7b002000 + (idx))

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static uint8_t g_buf[MAX_KEY_SIZE];

static size_t devstate_io(uint8_t *buf, size_t size, bool write)
{
	struct mtd_dev_s *mtd = px4_mtd_kernel_partition(MTD_DEVSTATE);
	struct mtd_geometry_s geo;
	size_t len;

	if (mtd == NULL || MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo) < 0 ||
	    geo.blocksize > sizeof(g_buf)) {
		return 0;
	}

	if (write) {
		if (size > geo.blocksize) {
			return 0;
		}

		memset(g_buf, 0xff, geo.blocksize);
		memcpy(g_buf, buf, size);
		return MTD_BWRITE(mtd, 0, 1, g_buf) == 1 ? size : 0;
	}

	if (MTD_BREAD(mtd, 0, 1, g_buf) != 1) {
		return 0;
	}

	len = g_buf[2] | (g_buf[3] << 8);

	if (len < 4 || len > geo.blocksize || (buf != NULL && len > size)) {
		return 0;
	}

	if (buf != NULL) {
		memcpy(buf, g_buf, len);
	}

	return len;
}

void keystore_init(void)
{
}

void keystore_deinit(void)
{
}

keystore_session_handle_t keystore_open(void)
{
	keystore_session_handle_t handle = { .handle = 1 };
	return handle;
}

void keystore_close(keystore_session_handle_t *handle)
{
	keystore_session_handle_init(handle);
}

size_t keystore_get_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key_buf, size_t key_buf_size)
{
	struct se05x_key_transmission_s data = {
		.entry = { .id = SE05X_KEYSTORE_ID(idx) },
		.content = { .buffer = g_buf, .buffer_size = sizeof(g_buf) },
	};
	size_t len = 0;

	if (!keystore_session_handle_valid(handle) || idx >= MAX_KEYS) {
		return 0;
	}

	nxmutex_lock(&g_lock);

	if (idx == DEVSTATE_KEY_IDX) {
		len = devstate_io(key_buf, key_buf_size, false);

	} else if (se05x_kioctl(SEIOC_GET_DATA, (unsigned long)&data) == 0) {
		len = data.content.buffer_content_size;

		if (key_buf != NULL) {
			if (len <= key_buf_size) {
				memcpy(key_buf, g_buf, len);

			} else {
				len = 0;
			}
		}
	}

	explicit_bzero(g_buf, sizeof(g_buf));
	nxmutex_unlock(&g_lock);
	return len;
}

bool keystore_put_key(keystore_session_handle_t handle, uint8_t idx, const uint8_t *key, size_t key_size)
{
	struct se05x_key_transmission_s data = {
		.entry = { .id = SE05X_KEYSTORE_ID(idx) },
		.content = { .buffer = g_buf, .buffer_size = key_size },
	};
	int ret;

	if (!keystore_session_handle_valid(handle) || idx >= MAX_KEYS || key == NULL || key_size == 0 ||
	    key_size > sizeof(g_buf)) {
		return false;
	}

	nxmutex_lock(&g_lock);

	if (idx == DEVSTATE_KEY_IDX) {
		ret = devstate_io((uint8_t *)key, key_size, true) == key_size ? 0 : -EIO;

	} else {
		memcpy(g_buf, key, key_size);
		se05x_kioctl(SEIOC_DELETE_KEY, SE05X_KEYSTORE_ID(idx));
		ret = se05x_kioctl(SEIOC_SET_DATA, (unsigned long)&data);
	}

	explicit_bzero(g_buf, sizeof(g_buf));
	nxmutex_unlock(&g_lock);
	return ret == 0;
}

bool keystore_modify_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key_buf, size_t key_buf_size,
			 keystore_callback_t cb, void *arg)
{
	return keystore_put_key(handle, idx, key_buf, key_buf_size);
}
