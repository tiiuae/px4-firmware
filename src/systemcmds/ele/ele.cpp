/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file ele.cpp
 *
 * Drive the identity key held inside the i.MX93 EdgeLock Enclave.
 *
 * The private half is generated in there with sign usage only and export
 * withheld, so the only things possible are reading the public half and
 * asking for a signature. Both go through the ordinary crypto API, which is
 * what carries them across into the kernel: this address space never holds a
 * key, only an index.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/crypto.h>

/* Above MAX_KEYS, so it cannot collide with a NOR keystore slot. */
#define ELE_IDENTITY_KEY_INDEX 0xe0
#define ELE_PUB_LEN            64
#define ELE_SIG_LEN            64

extern "C" __EXPORT int ele_main(int argc, char *argv[]);

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		"### Description\n"
		"The identity key inside the EdgeLock Enclave. Generated there with sign\n"
		"usage only and export withheld, so no command returns the private half.\n"
	);
	PRINT_MODULE_USAGE_NAME("ele", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pubkey", "Print the public half, creating the key on first use");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sign", "Sign a SHA-256 digest given as 64 hex characters");
}

static void print_hex(const char *label, const uint8_t *buf, size_t len)
{
	printf("%s ", label);

	for (size_t i = 0; i < len; i++) {
		printf("%02x", buf[i]);
	}

	printf("\n");
}

static int parse_hex(const char *in, uint8_t *out, size_t outlen)
{
	if (strlen(in) != outlen * 2) {
		return -EINVAL;
	}

	for (size_t i = 0; i < outlen; i++) {
		char byte[3] = { in[i * 2], in[i * 2 + 1], 0 };
		char *end = nullptr;
		long v = strtol(byte, &end, 16);

		if (end != byte + 2) {
			return -EINVAL;
		}

		out[i] = (uint8_t)v;
	}

	return 0;
}

static int cmd_pubkey()
{
	PX4Crypto crypto;
	uint8_t pub[ELE_PUB_LEN];
	size_t publen = sizeof(pub);

	if (!crypto.open(CRYPTO_ECDSA_P256)) {
		printf("ele: no P-256 crypto session\n");
		return 1;
	}

	if (!crypto.get_public_key(ELE_IDENTITY_KEY_INDEX, pub, &publen)) {
		printf("ele: the enclave would not give a public half\n");
		return 1;
	}

	print_hex("ele: pub", pub, publen);
	return 0;
}

static int cmd_sign(const char *hex)
{
	PX4Crypto crypto;
	uint8_t digest[32];
	uint8_t sig[ELE_SIG_LEN];

	if (parse_hex(hex, digest, sizeof(digest)) < 0) {
		printf("ele: a digest is 64 hex characters\n");
		return 1;
	}

	if (!crypto.open(CRYPTO_ECDSA_P256)) {
		printf("ele: no P-256 crypto session\n");
		return 1;
	}

	if (!crypto.sign(ELE_IDENTITY_KEY_INDEX, sig, digest, sizeof(digest))) {
		printf("ele: the enclave would not sign\n");
		return 1;
	}

	print_hex("ele: sig", sig, sizeof(sig));
	return 0;
}

int ele_main(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[1], "pubkey") == 0) {
		return cmd_pubkey();
	}

	if (argc == 3 && strcmp(argv[1], "sign") == 0) {
		return cmd_sign(argv[2]);
	}

	usage();
	return 1;
}
