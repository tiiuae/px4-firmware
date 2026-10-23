/****************************************************************************
 * The key-holding half of the Noise backend, for a board where no private or
 * transport key is readable from this address space.
 *
 * The static key is a keystore slot: the kernel performs the exchange and
 * returns the shared secret. Each transport key is a slot in the kernel's key
 * cache, used through a handle kept open for the life of the session.
 ****************************************************************************/

#include "noise_backend.h"

#include <px4_platform_common/crypto.h>

#include <string.h>

#if defined(PX4_CRYPTO) && defined(NOISE_STATIC_KEY_BY_INDEX)

static bool open_session(PX4Crypto &crypto)
{
	return crypto.open(CRYPTO_X25519);
}

#ifndef NOISE_HANDSHAKE_IN_KERNEL
extern "C" int noise_dh_static(const struct noise_static_key *s,
			       const uint8_t pk[NOISE_DHLEN],
			       uint8_t out[NOISE_DHLEN])
{
	PX4Crypto crypto;
	size_t len = NOISE_DHLEN;

	if (!open_session(crypto)) {
		return -1;
	}

	bool ok = crypto.key_agreement(s->index, pk, NOISE_DHLEN, out, &len)
		  && len == NOISE_DHLEN;
	crypto.close();

	if (!ok) {
		memset(out, 0, NOISE_DHLEN);
		return -1;
	}

	return 0;
}
#endif

extern "C" int noise_static_public(const struct noise_static_key *s,
				   uint8_t pk[NOISE_DHLEN])
{
	PX4Crypto crypto;
	size_t len = NOISE_DHLEN;

	if (!open_session(crypto)) {
		return -1;
	}

	bool ok = crypto.get_public_key(s->index, pk, &len) && len == NOISE_DHLEN;
	crypto.close();

	if (!ok) {
		memset(pk, 0, NOISE_DHLEN);
		return -1;
	}

	return 0;
}

#endif /* PX4_CRYPTO && NOISE_STATIC_KEY_BY_INDEX */

#if defined(PX4_CRYPTO) && defined(NOISE_SESSION_KEY_BY_INDEX)

extern "C" int noise_session_key_set(struct noise_session_key *k,
				     const uint8_t key[NOISE_KEYLEN])
{
	PX4Crypto *crypto = new PX4Crypto();

	if (crypto && crypto->open(CRYPTO_CHACHA20_POLY1305)) {
		for (uint8_t i = 0; i < CRYPTO_SESSION_KEY_COUNT; i++) {
			const uint8_t index = CRYPTO_SESSION_KEY_FIRST + i;

			if (crypto->set_key(0, nullptr, key, NOISE_KEYLEN, index)) {
				k->index = index;
				k->backend = crypto;
				return 0;
			}
		}
	}

	delete crypto;
	return -1;
}

extern "C" int noise_session_key_adopt(struct noise_session_key *k, uint8_t index)
{
	PX4Crypto *crypto = new PX4Crypto();

	if (crypto && crypto->open(CRYPTO_CHACHA20_POLY1305)) {
		k->index = index;
		k->backend = crypto;
		return 0;
	}

	delete crypto;
	return -1;
}

extern "C" void noise_session_key_clear(struct noise_session_key *k)
{
	PX4Crypto *crypto = static_cast<PX4Crypto *>(k->backend);

	if (crypto) {
		crypto->set_key(0, nullptr, nullptr, 0, k->index);
		delete crypto;
	}

	k->index = 0;
	k->backend = nullptr;
}

static PX4Crypto *with_nonce(const struct noise_session_key *k, uint64_t n)
{
	PX4Crypto *crypto = static_cast<PX4Crypto *>(k->backend);
	uint8_t iv[12] {};

	for (int i = 0; i < 8; i++) {
		iv[4 + i] = (uint8_t)(n >> (8 * i));
	}

	return crypto && crypto->renew_nonce(iv, sizeof(iv)) ? crypto : nullptr;
}

extern "C" int noise_session_encrypt(const struct noise_session_key *k,
				     uint64_t nonce, const uint8_t *pt,
				     size_t pt_len, uint8_t *out)
{
	PX4Crypto *crypto = with_nonce(k, nonce);
	size_t ct_len = pt_len;
	size_t tag_len = NOISE_TAGLEN;

	return crypto && crypto->encrypt_data(k->index, pt, pt_len, out, &ct_len,
					      out + pt_len, &tag_len) ? 0 : -1;
}

extern "C" int noise_session_decrypt(const struct noise_session_key *k,
				     uint64_t nonce, const uint8_t *ct,
				     size_t ct_len, uint8_t *out)
{
	if (ct_len < NOISE_TAGLEN) {
		return -1;
	}

	const size_t pt_len = ct_len - NOISE_TAGLEN;
	size_t out_len = pt_len;
	PX4Crypto *crypto = with_nonce(k, nonce);

	return crypto && crypto->decrypt_data(k->index, ct, pt_len, ct + pt_len,
					      NOISE_TAGLEN, out, &out_len) ? 0 : -1;
}

#endif /* PX4_CRYPTO && NOISE_SESSION_KEY_BY_INDEX */
