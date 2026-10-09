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
#include <fcntl.h>
#include <string.h>
#include <strings.h>

#include <nuttx/crypto/se05x.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>

#include "board_config.h"

#define SE05X_PATH "/dev/se05x"

static struct se05x_config_s g_config = {
	.address = PX4_I2C_OBDEV_SE050,
	.frequency = 400000,
};

static struct i2c_master_s *g_i2c;
static bool g_registered;

#ifdef CONFIG_DEV_SE05X_SCP03
#include "imxrt_caam.h"
#include <lib/crypto/crypto_utils/secure_heap.h>
#include <nuttx/mtd/mtd.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_mtd.h>

#define SLOT_SIZE  128
#define SLOT_MAGIC 0x53435033

struct keys_slot_s {
	uint32_t magic;
	uint32_t seq;
	uint8_t blob[sizeof(struct se05x_scp03_keys_s) + IMXRT_CAAM_BLOB_OVERHEAD];
};

_Static_assert(sizeof(struct keys_slot_s) <= SLOT_SIZE, "a key slot fits one FRAM block");

static const uint8_t g_keymod[IMXRT_CAAM_BLOB_KEYMOD] = "se05x-scp03-v1";
static struct se05x_scp03_keys_s g_live __attribute__((section(".secmem")));

static const struct se05x_scp03_keys_s g_defaults[] = {
	{
		.enc = {0x85, 0x2b, 0x59, 0x62, 0xe9, 0xcc, 0xe5, 0xd0, 0xbe, 0x74, 0x6b, 0x83, 0x3b, 0xcc, 0x62, 0x87},
		.mac = {0xdb, 0x0a, 0xa3, 0x19, 0xa4, 0x08, 0x69, 0x6c, 0x8e, 0x10, 0x7a, 0xb4, 0xe3, 0xc2, 0x6b, 0x47},
		.dek = {0x4c, 0x2f, 0x75, 0xc6, 0xa2, 0x78, 0xa4, 0xae, 0xe5, 0xc9, 0xaf, 0x7c, 0x50, 0xee, 0xa8, 0x0c},
	},
	{
		.enc = {0x88, 0xdb, 0xcd, 0x65, 0x82, 0x0d, 0x2a, 0xa0, 0x6f, 0xfa, 0xb9, 0x2a, 0xa8, 0xe7, 0x93, 0x64},
		.mac = {0xa8, 0x64, 0x4e, 0x2a, 0x04, 0xd9, 0xe9, 0xc8, 0xc0, 0xea, 0x60, 0x86, 0x68, 0x29, 0x99, 0xe5},
		.dek = {0x8a, 0x38, 0x72, 0x38, 0x99, 0x88, 0x18, 0x44, 0xe2, 0xc1, 0x51, 0x3d, 0xac, 0xd9, 0xf8, 0x0d},
	},
	{
		.enc = {0xbf, 0xc2, 0xdb, 0xe1, 0x82, 0x8e, 0x03, 0x5d, 0x3e, 0x7f, 0xa3, 0x6b, 0x90, 0x2a, 0x05, 0xc6},
		.mac = {0xbe, 0xf8, 0x5b, 0xd7, 0xba, 0x04, 0x97, 0xd6, 0x28, 0x78, 0x1c, 0xe4, 0x7b, 0x18, 0x8c, 0x96},
		.dek = {0xd8, 0x73, 0xf3, 0x16, 0xbe, 0x29, 0x7f, 0x2f, 0xc9, 0xc0, 0xe4, 0x5f, 0x54, 0x71, 0x06, 0x99},
	},
};

static int slot_io(int index, struct keys_slot_s *slot, bool write)
{
	struct mtd_dev_s *mtd = px4_mtd_kernel_partition(MTD_KEYS);
	uint8_t block[SLOT_SIZE];
	ssize_t n;

	if (mtd == NULL) {
		return -ENODEV;
	}

	if (write) {
		memset(block, 0xff, sizeof(block));
		memcpy(block, slot, sizeof(*slot));
		n = MTD_BWRITE(mtd, index, 1, block);

	} else {
		n = MTD_BREAD(mtd, index, 1, block);
		memcpy(slot, block, sizeof(*slot));
	}

	return n == 1 ? 0 : -EIO;
}

static int slot_open(int index, struct keys_slot_s *slot, struct se05x_scp03_keys_s *keys)
{
	if (slot_io(index, slot, false) < 0 || slot->magic != SLOT_MAGIC) {
		return -ENOENT;
	}

	return imxrt_caam_blob_decap(g_keymod, slot->blob, sizeof(*keys), (uint8_t *)keys);
}

static int keys_load(struct se05x_scp03_keys_s *keys)
{
	struct keys_slot_s slot[2];
	struct se05x_scp03_keys_s *tmp = sec_malloc(2 * sizeof(*tmp));
	bool valid[2];
	int count = 0;

	if (tmp == NULL) {
		return 0;
	}

	for (int i = 0; i < 2; i++) {
		valid[i] = slot_open(i, &slot[i], &tmp[i]) == 0;
	}

	int first = (valid[0] && valid[1]) ? (slot[1].seq > slot[0].seq) : (valid[1] ? 1 : 0);

	for (int i = 0; i < 2; i++) {
		int s = (first + i) % 2;

		if (valid[s]) {
			memcpy(&keys[count++], &tmp[s], sizeof(tmp[s]));
		}
	}

	explicit_bzero(tmp, 2 * sizeof(*tmp));
	sec_free(tmp);
	return count;
}

static int keys_store(const struct se05x_scp03_keys_s *keys, const struct se05x_scp03_keys_s *live)
{
	struct keys_slot_s slot[2];
	struct se05x_scp03_keys_s *tmp = sec_malloc(sizeof(*tmp));
	bool valid[2];
	bool holds_live[2];
	uint32_t seq = 0;

	if (tmp == NULL) {
		return -ENOMEM;
	}

	for (int i = 0; i < 2; i++) {
		valid[i] = slot_open(i, &slot[i], tmp) == 0;
		holds_live[i] = valid[i] && memcmp(tmp, live, sizeof(*tmp)) == 0;

		if (valid[i] && slot[i].seq >= seq) {
			seq = slot[i].seq + 1;
		}
	}

	explicit_bzero(tmp, sizeof(*tmp));
	sec_free(tmp);

	int target = holds_live[0] ? 1 : holds_live[1] ? 0 :
		     !valid[0] ? 0 : !valid[1] ? 1 : (slot[0].seq < slot[1].seq ? 0 : 1);
	struct keys_slot_s out;

	memset(&out, 0xff, sizeof(out));
	out.magic = SLOT_MAGIC;
	out.seq = seq;

	int ret = imxrt_caam_blob_encap(g_keymod, (const uint8_t *)keys, sizeof(*keys), out.blob);

	if (ret == 0) {
		ret = slot_io(target, &out, true);
	}

	return ret;
}

int board_se05x_rotate(const struct se05x_scp03_keys_s *keys)
{
	int ret = keys_store(keys, &g_live);

	if (ret < 0) {
		return ret;
	}

	ret = se05x_kioctl(SEIOC_ROTATE_SCP03, (unsigned long)keys);

	if (ret == 0) {
		memcpy(&g_live, keys, sizeof(g_live));
	}

	return ret;
}

static int try_register(const struct se05x_scp03_keys_s *keys)
{
	g_config.scp03 = keys;
	int ret = se05x_register(SE05X_PATH, g_i2c, &g_config);
	g_config.scp03 = NULL;

	if (ret == 0) {
		memcpy(&g_live, keys, sizeof(g_live));
		g_registered = true;
	}

	return ret;
}

int board_se05x_restore(const struct se05x_scp03_keys_s *keys)
{
	if (g_i2c == NULL) {
		return -ENODEV;
	}

	if (g_registered) {
		return -EALREADY;
	}

	int ret = try_register(keys);

	if (ret == 0) {
		ret = keys_store(keys, keys);
	}

	return ret;
}
#endif

int board_se05x_initialize(struct i2c_master_s *i2c)
{
	g_i2c = i2c;

	if (i2c == NULL) {
		return -ENODEV;
	}

#ifdef CONFIG_DEV_SE05X_SCP03
	secure_heap_init();
	g_config.zalloc = sec_malloc;
	g_config.free = sec_free;

	const size_t count = 2 + sizeof(g_defaults) / sizeof(g_defaults[0]);
	struct se05x_scp03_keys_s *keys = sec_malloc(count * sizeof(*keys));

	if (keys == NULL) {
		return -ENOMEM;
	}

	int n = keys_load(keys);
	int ret = -ENODEV;

	memcpy(&keys[n], g_defaults, sizeof(g_defaults));
	n += sizeof(g_defaults) / sizeof(g_defaults[0]);

	for (int i = 0; i < n && ret < 0; i++) {
		ret = try_register(&keys[i]);
	}

	explicit_bzero(keys, count * sizeof(*keys));
	sec_free(keys);
	return ret;
#else
	int ret = se05x_register(SE05X_PATH, i2c, &g_config);
	g_registered = ret == 0;
	return ret;
#endif
}
