/****************************************************************************
 *
 *   Copyright (c) 2020 Technology Innovation Institute. All rights reserved.
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
 * @file crypto.c
 *
 * Wrapper for the monocypher crypto
 *
 */

#include <inttypes.h>
#include <stdbool.h>

#include <px4_platform_common/arch_crypto.h>
#include <lib/crypto/monocypher/src/optional/monocypher-ed25519.h>

bool arch_signature_check(uint8_t  key_index,
			  px4_crypto_algorithm_t algorithm,
			  const uint8_t  *signature,
			  const uint8_t *message,
			  size_t message_size)
{
	bool ret;
	uint8_t public_key[64];
	size_t keylen = arch_get_key_by_index(key_index, public_key, sizeof(public_key));

	if (keylen == 0) {
		return false;
	}

	switch (algorithm) {
	case ED25519:
		ret = crypto_ed25519_check(signature, public_key, message, message_size) == 0 ? true : false;
		break;

	default:
		ret = false;
	}

	return ret;
}
