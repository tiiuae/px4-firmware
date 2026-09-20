/****************************************************************************
 * The static key half of the Noise backend, for a board where the private
 * key is not readable from this address space.
 *
 * The handle carries a keystore slot. The kernel performs the exchange and
 * returns the shared secret, so the private key is never here to leak.
 ****************************************************************************/

#include "noise_backend.h"

#include <px4_platform_common/crypto.h>

#include <string.h>

#if defined(PX4_CRYPTO) && defined(NOISE_STATIC_KEY_BY_INDEX)

static bool open_session(PX4Crypto &crypto)
{
	return crypto.open(CRYPTO_X25519);
}

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
