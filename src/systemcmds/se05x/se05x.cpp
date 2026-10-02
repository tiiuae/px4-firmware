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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/crypto/se05x.h>

static constexpr uint16_t SE05X_CONFIG_EDDSA = 0x0004;
static constexpr uint16_t SE05X_CONFIG_DH_MONT = 0x0008;
static constexpr uint32_t SE05X_TEST_KEY_ID = 0x7b000001;

static int ecdh_test(int fd)
{
	struct se05x_generate_keypair_s keypair {};
	keypair.id = SE05X_TEST_KEY_ID;
	keypair.cipher = SE05X_ASYM_CIPHER_EC_NIST_P_256;

	if (ioctl(fd, SEIOC_GENERATE_KEYPAIR, (unsigned long)&keypair) < 0) {
		PX4_ERR("generate P-256 key: %d", errno);
		return 1;
	}

	uint8_t secret[32];
	struct se05x_derive_key_s derive {};
	derive.private_key_id = SE05X_TEST_KEY_ID;
	derive.public_key_id = SE05X_TEST_KEY_ID;
	derive.content.buffer = secret;
	derive.content.buffer_size = sizeof(secret);

	int ret = ioctl(fd, SEIOC_DERIVE_SYMM_KEY, (unsigned long)&derive);
	int err = errno;
	explicit_bzero(secret, sizeof(secret));

	if (ret < 0) {
		PX4_ERR("ECDH refused: %d", err);

	} else {
		PX4_INFO_RAW("ECDH: %zu-byte shared secret\n", derive.content.buffer_content_size);
	}

	uint32_t id = SE05X_TEST_KEY_ID;

	if (ioctl(fd, SEIOC_DELETE_KEY, (unsigned long)id) < 0) {
		PX4_ERR("delete test key 0x%08" PRIx32 ": %d", id, errno);
		return 1;
	}

	return ret < 0 ? 1 : 0;
}

static void usage()
{
	PRINT_MODULE_DESCRIPTION("Read the identity of the SE05x secure element");
	PRINT_MODULE_USAGE_NAME("se05x", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Print the applet version and features, the unique id and the OEF id");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ecdh-test", "ECDH with a throwaway P-256 key inside the element, then delete it");
}

extern "C" __EXPORT int se05x_main(int argc, char *argv[])
{
	if (argc != 2 || (strcmp(argv[1], "info") != 0 && strcmp(argv[1], "ecdh-test") != 0)) {
		usage();
		return 1;
	}

	int fd = open("/dev/se05x", O_RDWR);

	if (fd < 0) {
		PX4_ERR("/dev/se05x: %d", errno);
		return 1;
	}

	if (strcmp(argv[1], "ecdh-test") == 0) {
		int ret = ecdh_test(fd);
		close(fd);
		return ret;
	}

	struct se05x_version_s version {};
	struct se05x_uid_s uid {};
	struct se05x_info_s info {};
	int ret = 1;

	if (ioctl(fd, SEIOC_GET_VERSION, (unsigned long)&version) < 0) {
		PX4_ERR("applet version: %d", errno);

	} else if (ioctl(fd, SEIOC_GET_UID, (unsigned long)&uid) < 0) {
		PX4_ERR("unique id: %d", errno);

	} else if (ioctl(fd, SEIOC_GET_INFO, (unsigned long)&info) < 0) {
		PX4_ERR("OEF id: %d", errno);

	} else {
		PX4_INFO_RAW("applet: %u.%u.%u, features 0x%04x, secure box 0x%04x\n", version.major, version.minor,
			     version.patch, version.applet_config, version.secure_box);
		PX4_INFO_RAW("X25519: %s, Ed25519: %s\n", (version.applet_config & SE05X_CONFIG_DH_MONT) ? "yes" : "no",
			     (version.applet_config & SE05X_CONFIG_EDDSA) ? "yes" : "no");
		PX4_INFO_RAW("unique id: ");

		for (size_t i = 0; i < sizeof(uid.uid); i++) {
			PX4_INFO_RAW("%02x", uid.uid[i]);
		}

		PX4_INFO_RAW("\nOEF id: 0x%04x\n", info.oef_id);
		ret = 0;
	}

	close(fd);
	return ret;
}
