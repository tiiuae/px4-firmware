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
#define ELE_AGREEMENT_KEY_INDEX 0xe2
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
	PRINT_MODULE_USAGE_COMMAND_DESCR("kex", "Key exchange probe: <peer public key, 128 hex> <case index>");
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

/* Mirrors struct ele_kex_result in the crypto backend. */
struct kex_result {
	uint32_t generate_rsp;
	uint32_t derive_id;
	uint32_t kex_rsp;
	uint32_t derived_key_id;
	uint32_t out_sz;
	uint8_t  derive_pub[64];
	uint8_t  out[32];
};

static int cmd_kex(const char *hex, const char *case_arg)
{
	PX4Crypto crypto;
	uint8_t peer[ELE_PUB_LEN] = {};
	kex_result res = {};
	size_t res_len = sizeof(res);

	if (hex != nullptr && parse_hex(hex, peer, sizeof(peer)) < 0) {
		printf("ele: a peer public key is 128 hex characters\n");
		return 1;
	}

	/* The case index rides in on the first word of the result buffer. */
	res.generate_rsp = case_arg != nullptr ? (uint32_t)strtoul(case_arg, nullptr, 10) : 0;

	if (!crypto.open(CRYPTO_ECDSA_P256)) {
		printf("ele: no P-256 crypto session\n");
		return 1;
	}

	if (!crypto.key_agreement(ELE_AGREEMENT_KEY_INDEX, peer, sizeof(peer),
				  (uint8_t *)&res, &res_len)) {
		printf("ele: key exchange probe failed to run\n");
		return 1;
	}

	printf("ele: generate 0x%08x id 0x%08x\n",
	       (unsigned)res.generate_rsp, (unsigned)res.derive_id);
	printf("ele: kex      0x%08x id 0x%08x out %u\n",
	       (unsigned)res.kex_rsp, (unsigned)res.derived_key_id,
	       (unsigned)res.out_sz);
	print_hex("ele: derive-pub", res.derive_pub, sizeof(res.derive_pub));
	print_hex("ele: out", res.out, sizeof(res.out));
	return 0;
}

int ele_main(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[1], "kex") == 0) {
		return cmd_kex(argc >= 3 ? argv[2] : nullptr,
			       argc >= 4 ? argv[3] : nullptr);
	}

	if (argc >= 2 && strcmp(argv[1], "pubkey") == 0) {
		return cmd_pubkey();
	}

	if (argc == 3 && strcmp(argv[1], "sign") == 0) {
		return cmd_sign(argv[2]);
	}

	usage();
	return 1;
}
