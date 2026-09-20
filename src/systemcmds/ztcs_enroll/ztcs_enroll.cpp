/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file ztcs_enroll.cpp
 *
 * Enrolment for the secure MAVLink link: reads the aircraft's link public key
 * out, and writes the station key and identity payload back in. The private
 * half never appears on this console.
 */

#include <lib/ztcs_secure_link/secure_link.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void usage(void)
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Enrols this aircraft on the secure MAVLink link.

Run `key` and `sign`, give both to `ztcs-mavlink-provision` on the ground,
then paste back the `write` line it prints. Both private keys are generated
on first use and neither is ever printed.

Once an operator key is pinned, storing a station key needs that operator's
signature over it, so a fielded aircraft cannot be pointed elsewhere.

### Examples
$ ztcs_enroll key
$ ztcs_enroll sign
$ ztcs_enroll operator <operator-public-hex>
$ ztcs_enroll write <station-public-hex> [signature-hex]
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("ztcs_enroll", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("key", "Print the link public key");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sign", "Sign the link key and print the identity");
	PRINT_MODULE_USAGE_COMMAND_DESCR("operator", "Pin the operator key, once");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write", "Store the station key");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Report what the keystore holds");
}

/* Strict on purpose: sscanf("%2x") would take a 0x prefix and a sign. */
static int nibble(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}

	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}

	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}

	return -1;
}

static bool from_hex(const char *in, uint8_t *out, size_t len)
{
	if (strlen(in) != len * 2) {
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		int hi = nibble(in[2 * i]);
		int lo = nibble(in[2 * i + 1]);

		if (hi < 0 || lo < 0) {
			return false;
		}

		out[i] = (uint8_t)((hi << 4) | lo);
	}

	return true;
}

static void print_hex(const uint8_t *b, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		printf("%02x", b[i]);
	}

	printf("\n");
}

static int cmd_key(void)
{
	struct secure_link_keys keys;
	uint8_t pub[NOISE_DHLEN];

	/* Generates the link key if this is the first run, which is the only
	 * reason to load a whole key set just to print one public value.
	 */
	(void)secure_link_ensure_keys(&keys);
	memset(&keys, 0, sizeof(keys));

	if (!secure_link_public_key(pub)) {
		PX4_ERR("no link key, and none could be generated");
		return 1;
	}

	print_hex(pub, sizeof(pub));
	return 0;
}

static int cmd_sign(void)
{
	uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN];

	if (!secure_link_self_sign(identity)) {
		return 1;
	}

	print_hex(identity, sizeof(identity));
	return 0;
}

static int cmd_operator(const char *operator_hex)
{
	uint8_t op[32];

	if (!from_hex(operator_hex, op, sizeof(op))) {
		PX4_ERR("operator key must be %d hex characters", (int)sizeof(op) * 2);
		return 1;
	}

	if (!secure_link_pin_operator(op)) {
		return 1;
	}

	PX4_INFO("operator key pinned");
	return 0;
}

static int cmd_write(const char *station_hex, const char *signature_hex)
{
	uint8_t station[NOISE_DHLEN];
	uint8_t signature[64];

	if (!from_hex(station_hex, station, sizeof(station))) {
		PX4_ERR("station key must be %d hex characters", (int)sizeof(station) * 2);
		return 1;
	}

	if (signature_hex != NULL && !from_hex(signature_hex, signature, sizeof(signature))) {
		PX4_ERR("signature must be %d hex characters", (int)sizeof(signature) * 2);
		return 1;
	}

	if (!secure_link_enroll(station, signature_hex != NULL ? signature : NULL)) {
		return 1;
	}

	PX4_INFO("enrolled");
	return 0;
}

static int cmd_status(void)
{
	struct secure_link_keys keys;
	bool enrolled = secure_link_ensure_keys(&keys);

	memset(&keys, 0, sizeof(keys));
	PX4_INFO("%s", enrolled ? "enrolled" : "not enrolled");
	return enrolled ? 0 : 1;
}

extern "C" __EXPORT int ztcs_enroll_main(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[1], "key") == 0) {
		return cmd_key();
	}

	if (argc >= 2 && strcmp(argv[1], "sign") == 0) {
		return cmd_sign();
	}

	if (argc == 3 && strcmp(argv[1], "operator") == 0) {
		return cmd_operator(argv[2]);
	}

	if ((argc == 3 || argc == 4) && strcmp(argv[1], "write") == 0) {
		return cmd_write(argv[2], argc == 4 ? argv[3] : NULL);
	}

	if (argc >= 2 && strcmp(argv[1], "status") == 0) {
		return cmd_status();
	}

	usage();
	return 1;
}
