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

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	NONE = 0,
	ED25519 = 1,
} px4_crypto_algorithm_t;

/*
 * Keystore access functions
 */

/* Get the length of the public key in keystore
 * idx: index of the key in keystore
 * returns the key length in bytes
 */
size_t arch_get_key_length(uint8_t idx);

/* Get a copy of the key from keystore
* idx: key index in keystore
* key: buffer to which the key is opied
* buf_size: length of the provided buffer
*/
size_t arch_get_key_by_index(uint8_t idx, uint8_t *key, size_t buf_size);

/*
 * Architecture specific PX4 Crypto API functions
 */

bool arch_signature_check(uint8_t  key_index,
			  px4_crypto_algorithm_t algorithm,
			  const uint8_t  *signature,
			  const uint8_t *message,
			  size_t message_size);
