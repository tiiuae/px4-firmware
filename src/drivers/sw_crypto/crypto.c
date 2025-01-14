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

#include <lib/crypto/monocypher/src/optional/monocypher-ed25519.h>
#include <px4_platform_common/crypto_backend.h>
#include <px4_random.h>
#include <tomcrypt.h>

extern void libtomcrypt_init(void);

/* room for 16 keys */
#define KEY_CACHE_LEN 16

#ifndef SECMEM_ALLOC
#define SECMEM_ALLOC XMALLOC
#endif

#ifndef SECMEM_FREE
#define SECMEM_FREE XFREE
#endif

#define SHA256_HASHLEN 32
#define OAEP_MAX_RSA_MODLEN 256 /* RSA2048 */
#define OAEP_MAX_MSGLEN (OAEP_MAX_RSA_MODLEN - 2 * SHA256_HASHLEN - 2)

/* crypto_(un)lock_aead */
/* Implementation of Monocypher crypto_lock_aead and crypto_unlock_aead requires these */
#define WIPE_BUFFER(buffer) crypto_wipe(buffer, sizeof(buffer))
static const uint8_t zero[128] = {0};
static size_t align(size_t x, size_t pow_2)
{
	return (~x + 1) & (pow_2 - 1);
}

static void store32_le(uint8_t out[4], uint32_t in)
{
	out[0] = in & 0xff;
	out[1] = (in >> 8) & 0xff;
	out[2] = (in >> 16) & 0xff;
	out[3] = (in >> 24) & 0xff;
}

static void store64_le(uint8_t out[8], uint64_t in)
{
	store32_le(out, (uint32_t) in);
	store32_le(out + 4, in >> 32);
}
static void lock_auth(uint8_t mac[16],
		      const uint8_t auth_key[32],
		      const uint8_t *ad,
		      size_t ad_size,
		      const uint8_t *cipher_text,
		      size_t text_size)
{
	uint8_t sizes[16]; // Not secret, not wiped
	store64_le(sizes + 0, ad_size);
	store64_le(sizes + 8, text_size);
	crypto_poly1305_ctx poly_ctx; // auto wiped...
	crypto_poly1305_init(&poly_ctx, auth_key);
	crypto_poly1305_update(&poly_ctx, ad, ad_size);
	crypto_poly1305_update(&poly_ctx, zero, align(ad_size, 16));
	crypto_poly1305_update(&poly_ctx, cipher_text, text_size);
	crypto_poly1305_update(&poly_ctx, zero, align(text_size, 16));
	crypto_poly1305_update(&poly_ctx, sizes, 16);
	crypto_poly1305_final(&poly_ctx, mac); // ...here
}
/* crypto_(un)lock_aead*/

/*
 * For now, this is just a dummy up/down counter for tracking open/close calls
 */
static int crypto_open_count = 0;

/*
 * Status of libtomcrypt initialization. This is a large library, which
 * is initialized & pulled in by linker only when it is actually used
 */
static bool tomcrypt_initialized = false;

typedef struct {
	size_t key_size;
	uint8_t *key;
} volatile_key_t;

static volatile_key_t key_cache[KEY_CACHE_LEN];

typedef struct {
	uint8_t nonce[24];
	uint64_t ctr;
} chacha20_context_t;

static inline void initialize_tomcrypt(void)
{
	if (!tomcrypt_initialized) {
		libtomcrypt_init();
		tomcrypt_initialized = true;
	}
}

/* Clear key cache */
static void clear_key_cache(void)
{
	for (int i = 0; i < KEY_CACHE_LEN; i++) {
		SECMEM_FREE(key_cache[i].key);
		key_cache[i].key = NULL;
		key_cache[i].key_size = 0;
	}
}

/* Retrieve a direct pointer to the cached temporary/public key */
static const uint8_t *crypto_get_key_ptr(keystore_session_handle_t handle,
		uint8_t key_idx,
		size_t *len)
{
	uint8_t *ret;

	if (key_idx >= KEY_CACHE_LEN) {
		*len = 0;
		return NULL;
	}

	ret = key_cache[key_idx].key;

	/* if the key doesn't exist in the key cache, try to read it in there from keystore */
	if (ret == NULL) {
		/* First check if the key exists in the keystore and retrieve its length */
		*len = keystore_get_key(handle, key_idx, NULL, 0);

		if (*len > 0) {
			/* Allocate memory for the key in the cache */
			ret = SECMEM_ALLOC(*len);

			/* Retrieve the key from the keystore */
			if (ret) {
				if (keystore_get_key(handle, key_idx, ret, *len) > 0) {
					/* Success, store the key in cache */
					key_cache[key_idx].key_size = *len;
					key_cache[key_idx].key = ret;

				} else {
					/* key retrieval failed, free the memory */
					SECMEM_FREE(ret);
				}
			}
		}
	}

	*len = key_cache[key_idx].key_size;

	return ret;
}

void crypto_init()
{
	keystore_init();
	clear_key_cache();
}

void crypto_deinit()
{
	keystore_deinit();
}

crypto_session_handle_t crypto_open(px4_crypto_algorithm_t algorithm)
{
	crypto_session_handle_t ret;
	ret.keystore_handle = keystore_open();

	if (keystore_session_handle_valid(ret.keystore_handle)) {
		ret.algorithm = algorithm;
		ret.handle = ++crypto_open_count;

	} else {
		ret.handle = 0;
		ret.context = NULL;
		ret.algorithm = CRYPTO_NONE;
		return ret;
	}

	switch (algorithm) {
	case CRYPTO_XCHACHA20: {
			chacha20_context_t *context = XMALLOC(sizeof(chacha20_context_t));

			if (!context) {
				ret.handle = 0;
				crypto_open_count--;

			} else {
				ret.context = context;
				px4_get_secure_random(context->nonce, sizeof(context->nonce));
				context->ctr = 0;
			}
		}
		break;

	default:
		ret.context = NULL;
	}

	return ret;
}

void crypto_close(crypto_session_handle_t *handle)
{
	// Not open?
	if (!crypto_session_handle_valid(*handle)) {
		return;
	}

	crypto_open_count--;
	handle->handle = 0;
	keystore_close(&handle->keystore_handle);
	XFREE(handle->context);
	handle->context = NULL;
}

bool crypto_signature_gen(crypto_session_handle_t handle,
			  uint8_t key_index,
			  uint8_t *signature,
			  const uint8_t *message,
			  size_t message_size)
{
	bool ret = false;
	size_t keylen = 0;
	const uint8_t *private_key = NULL;

	if (crypto_session_handle_valid(handle)) {
		private_key = crypto_get_key_ptr(handle.keystore_handle, key_index, &keylen);
	}

	if (keylen == 0 || private_key == NULL) {
		return false;
	}

	switch (handle.algorithm) {
	case CRYPTO_ED25519:
		if (keylen >= 32) {
			/* In the DER format ed25519 key the raw private key part is always the last 32 bytes.
			 * This simple "parsing" works for both "raw" key and DER format
			 */
			private_key += keylen - 32;
			crypto_ed25519_sign(signature, private_key, 0, message, message_size);
			ret = (signature != NULL);
		}

		break;

	default:
		ret = false;
	}

	return ret;
}

bool crypto_signature_check(crypto_session_handle_t handle,
			    uint8_t key_index,
			    const uint8_t *signature,
			    const uint8_t *message,
			    size_t message_size)
{
	bool ret = false;
	size_t keylen = 0;
	const uint8_t *public_key = NULL;

	if (crypto_session_handle_valid(handle)) {
		public_key = crypto_get_key_ptr(handle.keystore_handle, key_index, &keylen);
	}

	if (keylen == 0 || public_key == NULL) {
		return false;
	}

	switch (handle.algorithm) {
	case CRYPTO_ED25519:
		if (keylen >= 32) {
			/* In the DER format ed25510 key the raw public key part is always the last 32 bytes.
			 * This simple "parsing" works for both "raw" key and DER format
			 */
			public_key += keylen - 32;
			ret = crypto_ed25519_check(signature, public_key, message, message_size) == 0;
		}

		break;

	case CRYPTO_RSA_SIG: {
			rsa_key key;

			initialize_tomcrypt();

			if (rsa_import(public_key, keylen, &key) == CRYPT_OK) {
				// Register hash algorithm.
				const struct ltc_hash_descriptor *hash_desc = &sha256_desc;
				const int hash_idx = register_hash(hash_desc);

				if (hash_idx >= 0) {
					// Hash message.
					unsigned char hash[32];
					hash_state md;

					if (hash_desc->init(&md) == CRYPT_OK
					    && hash_desc->process(&md,
								  (const unsigned char *) message,
								  (unsigned long) message_size)
					    == CRYPT_OK
					    && hash_desc->done(&md, hash) == CRYPT_OK) {
						// Define padding scheme.
						const int padding = LTC_PKCS_1_V1_5;
						const unsigned long saltlen = 0;

						// Verify signature.
						int stat = 0;

						if (rsa_verify_hash_ex(signature,
								       256,
								       hash,
								       hash_desc->hashsize,
								       padding,
								       hash_idx,
								       saltlen,
								       &stat,
								       &key)
						    == CRYPT_OK
						    && stat) {
							ret = true;
						}
					}

					// Clean up.
					memset(hash, 0, sizeof(hash));
					memset(&md, 0, sizeof(md));
					unregister_hash(hash_desc);
				}

				// Free RSA key.
				rsa_free(&key);
			}
		}
		break;

	default:
		ret = false;
		break;
	}

	return ret;
}

bool crypto_encrypt_data(crypto_session_handle_t handle,
			 const uint8_t key_idx,
			 const uint8_t *message,
			 const size_t message_size,
			 uint8_t *cipher,
			 size_t *cipher_size,
			 uint8_t *mac,
			 size_t *mac_size)
{
	bool ret = false;

	if (!crypto_session_handle_valid(handle)) {
		return ret;
	}

	switch (handle.algorithm) {
	case CRYPTO_NONE:
		if (*cipher_size >= message_size) {
			/* In-place there is no copy needed */
			if (message != cipher) {
				memcpy(cipher, message, message_size);
			}

			*cipher_size = message_size;
			ret = true;
		}

		break;

	case CRYPTO_XCHACHA20: {
			size_t key_sz;
			uint8_t *key = (uint8_t *) crypto_get_key_ptr(handle.keystore_handle, key_idx, &key_sz);
			chacha20_context_t *context = handle.context;

			if (key_sz == 32 && *mac_size >= 16 && *cipher_size >= message_size) {
				// Encrypt the data
				uint8_t sub_key[32];
				crypto_hchacha20(sub_key, key, context->nonce);
				context->ctr = crypto_chacha20_ctr(cipher,
								   message,
								   message_size,
								   sub_key,
								   context->nonce + 16,
								   context->ctr + 1);

				if (mac) {
					uint8_t auth_key[64]; // "Wasting" the whole Chacha block is faster
					crypto_chacha20(auth_key, 0, 64, sub_key, context->nonce + 16);
					lock_auth(mac, auth_key, NULL, 0, cipher, message_size);
					WIPE_BUFFER(auth_key);
					*mac_size = 16;
				}

				WIPE_BUFFER(sub_key);
				*cipher_size = message_size;
				ret = true;
			}
		}
		break;

	case CRYPTO_RSA_OAEP: {
			rsa_key key;
			size_t key_sz;
			unsigned long outlen = *cipher_size;
			uint8_t *public_key = (uint8_t *) crypto_get_key_ptr(handle.keystore_handle,
					      key_idx,
					      &key_sz);
			*cipher_size = 0;

			initialize_tomcrypt();

			if (public_key && rsa_import(public_key, key_sz, &key) == CRYPT_OK) {
				if (outlen >= ltc_mp.unsigned_size(key.N)
				    && pkcs_1_oaep_encode(message,
							  message_size,
							  NULL,
							  0,
							  ltc_mp.count_bits(key.N),
							  NULL,
							  0,
							  0,
							  cipher,
							  &outlen)
				    == CRYPT_OK
				    && ltc_mp.rsa_me(cipher, outlen, cipher, &outlen, PK_PUBLIC, &key) == CRYPT_OK) {
					*cipher_size = outlen;
					ret = true;
				}

				rsa_free(&key);
			}
		}
		break;

	default:
		break;
	}

	return ret;
}

bool crypto_generate_keypair(crypto_session_handle_t handle,
			     size_t key_size,
			     uint8_t key_idx,
			     bool persistent)
{
	/* unimplemented */
	return false;
}

bool crypto_generate_key(crypto_session_handle_t handle, uint8_t idx, bool persistent)
{
	bool ret = false;

	if (idx >= KEY_CACHE_LEN) {
		return false;
	}

	switch (handle.algorithm) {
	case CRYPTO_XCHACHA20:
		if (key_cache[idx].key_size < 32) {
			if (key_cache[idx].key_size > 0) {
				SECMEM_FREE(key_cache[idx].key);
				key_cache[idx].key_size = 0;
			}

			key_cache[idx].key = SECMEM_ALLOC(32);
		}

		if (key_cache[idx].key) {
			key_cache[idx].key_size = 32;
			px4_get_secure_random(key_cache[idx].key, 32);
			ret = true;

		} else {
			key_cache[idx].key_size = 0;
		}

		break;

	default:
		break;
	}

	if (ret && persistent) {
		keystore_put_key(handle.keystore_handle, idx, key_cache[idx].key, key_cache[idx].key_size);
	}

	return ret;
}

bool crypto_get_encrypted_key(crypto_session_handle_t handle,
			      uint8_t key_idx,
			      uint8_t *key,
			      size_t *max_len,
			      uint8_t encryption_key_idx)
{
	// Retrieve the plaintext key
	bool ret = true;
	size_t key_sz;
	const uint8_t *plain_key = crypto_get_key_ptr(handle.keystore_handle, key_idx, &key_sz);

	if (key_sz == 0) {
		return false;
	}

	// Encrypt it
	if (key != NULL) {
		size_t mac_size = 0;
		ret = crypto_encrypt_data(handle,
					  encryption_key_idx,
					  plain_key,
					  key_sz,
					  key,
					  max_len,
					  NULL,
					  &mac_size);

	} else {
		switch (handle.algorithm) {
		case CRYPTO_RSA_OAEP:
			/* The length is the RSA key modulus length, and the maximum plaintext
			 * length is calculated from that. This is now just fixed for RSA2048,
			 * but one could also parse the RSA key
			 * (encryption_key_idx) here and calculate the lengths.
			 */

			*max_len = key_sz <= OAEP_MAX_MSGLEN ? OAEP_MAX_RSA_MODLEN : 0;
			ret = true;
			break;

		default:
			*max_len = 0;
			break;
		}
	}

	return ret;
}

bool crypto_get_nonce(crypto_session_handle_t handle, uint8_t *nonce, size_t *nonce_len)
{
	switch (handle.algorithm) {
	case CRYPTO_XCHACHA20: {
			chacha20_context_t *context = handle.context;

			if (nonce != NULL && context != NULL) {
				memcpy(nonce, context->nonce, sizeof(context->nonce));
			}

			*nonce_len = sizeof(context->nonce);
		}
		break;

	default:
		*nonce_len = 0;
	}

	return true;
}

size_t crypto_get_min_blocksize(crypto_session_handle_t handle, uint8_t key_idx)
{
	size_t ret;

	switch (handle.algorithm) {
	case CRYPTO_XCHACHA20:
		ret = 64;
		break;

	default:
		ret = 1;
	}

	return ret;
}

bool crypto_renew_nonce(crypto_session_handle_t handle, const uint8_t *nonce, size_t nonce_size)
{
	bool ret = false;

	if (!handle.context) {
		return ret;
	}

	switch (handle.algorithm) {
	case CRYPTO_XCHACHA20: {
			chacha20_context_t *context = handle.context;

			if (nonce_size != sizeof(context->nonce)) {
				ret = false;
				break;
			}

			if (nonce != NULL) {
				memcpy(context->nonce, nonce, sizeof(context->nonce));
				ret = true;

			} else {
				px4_get_secure_random(context->nonce, sizeof(context->nonce));
				ret = true;
			}

			context->ctr = 0;
			ret = true;
		}
		break;

	default:
		break;
	}

	return ret;
}

bool crypto_decrypt_data(crypto_session_handle_t handle,
			 uint8_t key_index,
			 const uint8_t *cipher,
			 const size_t cipher_size,
			 const uint8_t *mac,
			 const size_t mac_size,
			 uint8_t *message,
			 size_t *message_size)
{
	bool ret = false;

	if (!crypto_session_handle_valid(handle)) {
		return ret;
	}

	switch (handle.algorithm) {
	case CRYPTO_NONE:
		if (*message_size >= cipher_size) {
			/* In-place there is no copy needed */
			if (cipher != message) {
				memcpy(message, cipher, cipher_size);
			}

			*message_size = cipher_size;
			ret = true;
		}

		break;

	case CRYPTO_XCHACHA20: {
			size_t key_sz;
			uint8_t *key = (uint8_t *) crypto_get_key_ptr(handle.keystore_handle, key_index, &key_sz);
			chacha20_context_t *context = handle.context;

			if (key_sz == 32 && mac_size == 16 && *message_size >= cipher_size) {
				uint8_t sub_key[32];
				crypto_hchacha20(sub_key, key, context->nonce);
				bool mac_verified = false;

				if (mac) {
					uint8_t auth_key[64]; // "Wasting" the whole Chacha block is faster
					crypto_chacha20(auth_key, 0, 64, sub_key, context->nonce + 16);
					uint8_t real_mac[16];
					lock_auth(real_mac, auth_key, NULL, 0, cipher, cipher_size);
					WIPE_BUFFER(auth_key);

					if (!crypto_verify16(mac, real_mac)) {
						mac_verified = true;
					}

					WIPE_BUFFER(real_mac);
				}

				if (!mac || mac_verified) {
					context->ctr = crypto_chacha20_ctr(message,
									   cipher,
									   cipher_size,
									   sub_key,
									   context->nonce + 16,
									   context->ctr + 1);
					ret = true;
					*message_size = cipher_size;
				}

				WIPE_BUFFER(sub_key);
			}
		}

		break;

	default:
		break;
	}

	return ret;
}
