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
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <nuttx/crypto/se05x.h>

static constexpr uint16_t SE05X_CONFIG_EDDSA = 0x0004;
static constexpr uint16_t SE05X_CONFIG_DH_MONT = 0x0008;
static constexpr uint32_t SE05X_TEST_KEY_ID = 0x7b000001;
static constexpr uint32_t SE05X_IDENTITY_KEY_ID = 0x7b000010;
static constexpr uint8_t SE05X_ED25519_TEST_MESSAGE[] = "se05x ed25519 test";

static void print_hex(const char *label, const uint8_t *buf, size_t len)
{
	PX4_INFO_RAW("%s: ", label);

	for (size_t i = 0; i < len; i++) {
		PX4_INFO_RAW("%02x", buf[i]);
	}

	PX4_INFO_RAW("\n");
}

static bool parse_hex32(const char *hex, uint8_t out[32])
{
	if (strlen(hex) != 64) {
		return false;
	}

	for (size_t i = 0; i < 32; i++) {
		unsigned byte;

		if (sscanf(hex + 2 * i, "%2x", &byte) != 1) {
			return false;
		}

		out[i] = byte;
	}

	return true;
}

static bool fresh_test_key(int fd, se05x_asym_cipher_type_e cipher, const char *label)
{
	struct se05x_generate_keypair_s keypair {};
	keypair.id = SE05X_TEST_KEY_ID;
	keypair.cipher = cipher;

	ioctl(fd, SEIOC_DELETE_KEY, (unsigned long)SE05X_TEST_KEY_ID);

	if (ioctl(fd, SEIOC_GENERATE_KEYPAIR, (unsigned long)&keypair) < 0) {
		PX4_ERR("generate %s key: %d", label, errno);
		return false;
	}

	return true;
}

static bool test_key(int fd, se05x_asym_cipher_type_e cipher, const char *label)
{
	if (!fresh_test_key(fd, cipher, label)) {
		return false;
	}

	uint8_t point[32];
	struct se05x_key_transmission_s key {};
	key.entry.id = SE05X_TEST_KEY_ID;
	key.entry.cipher = cipher;
	key.content.buffer = point;
	key.content.buffer_size = sizeof(point);

	if (ioctl(fd, SEIOC_GET_KEY, (unsigned long)&key) < 0) {
		PX4_ERR("read %s public key: %d", label, errno);
		return false;
	}

	print_hex("public", point, sizeof(point));
	return true;
}

static int delete_test_key(int fd, int ret)
{
	uint32_t id = SE05X_TEST_KEY_ID;

	if (ioctl(fd, SEIOC_DELETE_KEY, (unsigned long)id) < 0) {
		PX4_ERR("delete test key 0x%08" PRIx32 ": %d", id, errno);
		return 1;
	}

	return ret;
}

static int ed25519_test(int fd)
{
	if (!test_key(fd, SE05X_ASYM_CIPHER_EC_ED25519, "Ed25519")) {
		return delete_test_key(fd, 1);
	}

	uint8_t sig[64];
	struct se05x_signature_s signature {};
	signature.key_id = SE05X_TEST_KEY_ID;
	signature.algorithm = SE05X_ALGORITHM_ED25519;
	signature.tbs.buffer = (uint8_t *)SE05X_ED25519_TEST_MESSAGE;
	signature.tbs.buffer_size = sizeof(SE05X_ED25519_TEST_MESSAGE) - 1;
	signature.tbs.buffer_content_size = sizeof(SE05X_ED25519_TEST_MESSAGE) - 1;
	signature.signature.buffer = sig;
	signature.signature.buffer_size = sizeof(sig);

	const hrt_abstime start = hrt_absolute_time();
	int ret = ioctl(fd, SEIOC_CREATE_SIGNATURE, (unsigned long)&signature);
	const hrt_abstime elapsed = hrt_elapsed_time(&start);

	if (ret < 0) {
		PX4_ERR("Ed25519 sign: %d", errno);
		return delete_test_key(fd, 1);
	}

	PX4_INFO_RAW("message: %s\n", SE05X_ED25519_TEST_MESSAGE);
	print_hex("signature", sig, signature.signature.buffer_content_size);
	PX4_INFO_RAW("signed in %" PRIu64 " us\n", elapsed);
	return delete_test_key(fd, 0);
}

static int x25519_test(int fd, const char *hex)
{
	uint8_t peer[32];

	if (!parse_hex32(hex, peer)) {
		PX4_ERR("want the peer's X25519 public key as 64 hex digits");
		return 1;
	}

	if (!test_key(fd, SE05X_ASYM_CIPHER_EC_X25519, "X25519")) {
		return delete_test_key(fd, 1);
	}

	uint8_t secret[32];
	struct se05x_derive_key_s derive {};
	derive.private_key_id = SE05X_TEST_KEY_ID;
	derive.public_key.buffer = peer;
	derive.public_key.buffer_size = sizeof(peer);
	derive.public_key.buffer_content_size = sizeof(peer);
	derive.content.buffer = secret;
	derive.content.buffer_size = sizeof(secret);

	const hrt_abstime start = hrt_absolute_time();
	int ret = ioctl(fd, SEIOC_DERIVE_SYMM_KEY, (unsigned long)&derive);
	const hrt_abstime elapsed = hrt_elapsed_time(&start);

	if (ret < 0) {
		PX4_ERR("X25519: %d", errno);
		return delete_test_key(fd, 1);
	}

	print_hex("shared", secret, derive.content.buffer_content_size);
	explicit_bzero(secret, sizeof(secret));
	PX4_INFO_RAW("derived in %" PRIu64 " us\n", elapsed);
	return delete_test_key(fd, 0);
}

static int identity(int fd)
{
	struct se05x_generate_keypair_s keypair {};
	keypair.id = SE05X_IDENTITY_KEY_ID;
	keypair.cipher = SE05X_ASYM_CIPHER_EC_NIST_P_256;

	if (ioctl(fd, SEIOC_GENERATE_KEYPAIR, (unsigned long)&keypair) == 0) {
		PX4_INFO_RAW("identity key generated\n");

	} else if (errno != EEXIST) {
		PX4_ERR("generate identity key: %d", errno);
		return 1;
	}

	uint8_t point[65];
	struct se05x_key_transmission_s key {};
	key.entry.id = SE05X_IDENTITY_KEY_ID;
	key.entry.cipher = SE05X_ASYM_CIPHER_EC_NIST_P_256;
	key.content.buffer = point;
	key.content.buffer_size = sizeof(point);

	if (ioctl(fd, SEIOC_GET_KEY, (unsigned long)&key) < 0) {
		PX4_ERR("read identity public key: %d", errno);
		return 1;
	}

	if (key.content.buffer_content_size != sizeof(point)) {
		PX4_ERR("identity public key is %zu bytes, want 65", key.content.buffer_content_size);
		return 1;
	}

	PX4_INFO_RAW("key id: 0x%08" PRIx32 "\n", SE05X_IDENTITY_KEY_ID);
	print_hex("public", point, sizeof(point));
	return 0;
}

static int sign(int fd, const char *hex)
{
	uint8_t digest[32];

	if (!parse_hex32(hex, digest)) {
		PX4_ERR("want a SHA-256 digest as 64 hex digits");
		return 1;
	}

	uint8_t der[72];
	struct se05x_signature_s signature {};
	signature.key_id = SE05X_IDENTITY_KEY_ID;
	signature.algorithm = SE05X_ALGORITHM_SHA256;
	signature.tbs.buffer = digest;
	signature.tbs.buffer_size = sizeof(digest);
	signature.tbs.buffer_content_size = sizeof(digest);
	signature.signature.buffer = der;
	signature.signature.buffer_size = sizeof(der);

	const hrt_abstime start = hrt_absolute_time();
	int ret = ioctl(fd, SEIOC_CREATE_SIGNATURE, (unsigned long)&signature);
	const hrt_abstime elapsed = hrt_elapsed_time(&start);

	if (ret < 0) {
		PX4_ERR("sign: %d", errno);
		return 1;
	}

	print_hex("signature", der, signature.signature.buffer_content_size);
	PX4_INFO_RAW("signed in %" PRIu64 " us\n", elapsed);
	return 0;
}

static int ecdh_test(int fd)
{
	if (!fresh_test_key(fd, SE05X_ASYM_CIPHER_EC_NIST_P_256, "P-256")) {
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

	return delete_test_key(fd, ret < 0 ? 1 : 0);
}

static int info(int fd)
{
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

	return ret;
}

static void usage()
{
	PRINT_MODULE_DESCRIPTION("Read the SE05x secure element and use its identity key");
	PRINT_MODULE_USAGE_NAME("se05x", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Print the applet version and features, the unique id and the OEF id");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ecdh-test", "ECDH with a throwaway P-256 key inside the element, then delete it");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ed25519-test", "Sign a fixed message with a throwaway Ed25519 key, then delete it");
	PRINT_MODULE_USAGE_COMMAND_DESCR("x25519-test", "X25519 of a throwaway key and a peer key, then delete it");
	PRINT_MODULE_USAGE_ARG("<peer>", "64 hex digits", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("identity", "Generate the P-256 identity key on first use, print its public point");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sign", "Sign a SHA-256 digest with the identity key, print the signature and time");
	PRINT_MODULE_USAGE_ARG("<digest>", "64 hex digits", false);
}

extern "C" __EXPORT int se05x_main(int argc, char *argv[])
{
	if (argc < 2 || argc != (strcmp(argv[1], "sign") == 0 || strcmp(argv[1], "x25519-test") == 0 ? 3 : 2)) {
		usage();
		return 1;
	}

	int fd = open("/dev/se05x", O_RDWR);

	if (fd < 0) {
		PX4_ERR("/dev/se05x: %d", errno);
		return 1;
	}

	int ret = 1;

	if (strcmp(argv[1], "info") == 0) {
		ret = info(fd);

	} else if (strcmp(argv[1], "ecdh-test") == 0) {
		ret = ecdh_test(fd);

	} else if (strcmp(argv[1], "ed25519-test") == 0) {
		ret = ed25519_test(fd);

	} else if (strcmp(argv[1], "x25519-test") == 0) {
		ret = x25519_test(fd, argv[2]);

	} else if (strcmp(argv[1], "identity") == 0) {
		ret = identity(fd);

	} else if (strcmp(argv[1], "sign") == 0) {
		ret = sign(fd, argv[2]);

	} else {
		usage();
	}

	close(fd);
	return ret;
}
