/****************************************************************************
 * Key material for the link, from the PX4 keystore.
 *
 * The static private key is generated here and never leaves. Only the
 * public half is ever read out, for enrolment to sign.
 ****************************************************************************/

#include "secure_link.h"

#include <px4_platform_common/log.h>

#include <string.h>

#if defined(PX4_CRYPTO)

#include <px4_platform_common/crypto_backend.h>

/* imx9_keystore holds 50 slots. Read-only is a per-key flag rather than a
 * range: a slot takes a write until something marks it so, and a key
 * provisioned from a PC is marked on its next write. The logger holds 1 and 2.
 */
#ifndef ZTCS_KEY_SLOT_STATION_PUBLIC
#define ZTCS_KEY_SLOT_STATION_PUBLIC 3
#endif
#ifndef ZTCS_KEY_SLOT_STATIC_PRIVATE
#define ZTCS_KEY_SLOT_STATIC_PRIVATE 15
#endif
#ifndef ZTCS_KEY_SLOT_IDENTITY
#define ZTCS_KEY_SLOT_IDENTITY 16
#endif

/* A short read would leave the tail of the buffer as whatever was there. */
static bool read_slot(keystore_session_handle_t ks, uint8_t idx,
		      uint8_t *out, size_t expect)
{
	return keystore_get_key(ks, idx, out, expect) == expect;
}

static bool all_zero(const uint8_t *b, size_t len)
{
	uint8_t acc = 0;

	for (size_t i = 0; i < len; i++) {
		acc |= b[i];
	}

	return acc == 0;
}

/* An unseeded pool returns zeros, or the same bytes every time, and both
 * look like a key. Refusing matters more than generating: the bad key works
 * and nothing downstream can tell.
 */
static bool draw_static_key(uint8_t out[NOISE_DHLEN])
{
	uint8_t probe[NOISE_DHLEN];
	bool ok = noise_random(out, NOISE_DHLEN) == 0
		  && noise_random(probe, NOISE_DHLEN) == 0
		  && !all_zero(out, NOISE_DHLEN)
		  && memcmp(out, probe, NOISE_DHLEN) != 0;

	if (!ok) {
		PX4_ERR("entropy not usable; refusing to generate a key");
		memset(out, 0, NOISE_DHLEN);
	}

	memset(probe, 0, sizeof(probe));
	return ok;
}

bool secure_link_ensure_keys(struct secure_link_keys *keys)
{
	keystore_session_handle_t ks = keystore_open();
	bool ok = false;

	if (!keystore_session_handle_valid(ks)) {
		PX4_ERR("cannot open the keystore");
		memset(keys, 0, sizeof(*keys));
		return false;
	}

	if (!read_slot(ks, ZTCS_KEY_SLOT_STATIC_PRIVATE, keys->static_private, NOISE_DHLEN)) {
		if (!draw_static_key(keys->static_private)
		    || !keystore_put_key(ks, ZTCS_KEY_SLOT_STATIC_PRIVATE,
					 keys->static_private, NOISE_DHLEN)) {
			PX4_ERR("could not establish a static key");
			goto out;
		}

		PX4_INFO("static key generated");
	}

	/* Enrolment signs the public half of the key above, so on a first boot
	 * these are legitimately absent.
	 */
	if (!read_slot(ks, ZTCS_KEY_SLOT_STATION_PUBLIC, keys->station_public, NOISE_DHLEN)
	    || !read_slot(ks, ZTCS_KEY_SLOT_IDENTITY, keys->identity,
			  NOISE_IDENTITY_PAYLOAD_LEN)) {
		PX4_WARN("not enrolled yet: no station key or identity");
		goto out;
	}

	ok = true;

out:
	keystore_close(&ks);

	/* A half-loaded key set still looks like keys. */
	if (!ok) {
		memset(keys, 0, sizeof(*keys));
	}

	return ok;
}

bool secure_link_public_key(uint8_t out[NOISE_DHLEN])
{
	keystore_session_handle_t ks = keystore_open();
	uint8_t priv[NOISE_DHLEN];
	bool ok;

	if (!keystore_session_handle_valid(ks)) {
		memset(out, 0, NOISE_DHLEN);
		return false;
	}

	ok = read_slot(ks, ZTCS_KEY_SLOT_STATIC_PRIVATE, priv, NOISE_DHLEN);
	keystore_close(&ks);

	if (ok) {
		noise_dh_public(priv, out);

	} else {
		memset(out, 0, NOISE_DHLEN);
	}

	memset(priv, 0, sizeof(priv));
	return ok;
}

bool secure_link_enroll(const uint8_t station_public[NOISE_DHLEN],
			const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN])
{
	keystore_session_handle_t ks = keystore_open();
	bool ok;

	if (!keystore_session_handle_valid(ks)) {
		PX4_ERR("cannot open the keystore");
		return false;
	}

	ok = keystore_put_key(ks, ZTCS_KEY_SLOT_STATION_PUBLIC, station_public, NOISE_DHLEN)
	     && keystore_put_key(ks, ZTCS_KEY_SLOT_IDENTITY, identity,
				 NOISE_IDENTITY_PAYLOAD_LEN);
	keystore_close(&ks);

	if (!ok) {
		PX4_ERR("could not write the keystore; a slot may be marked read only");
	}

	return ok;
}

#else /* PX4_CRYPTO */

bool secure_link_ensure_keys(struct secure_link_keys *keys)
{
	memset(keys, 0, sizeof(*keys));
	PX4_ERR("no keystore on this board: enable the PX4 crypto backend");
	return false;
}

bool secure_link_public_key(uint8_t out[NOISE_DHLEN])
{
	memset(out, 0, NOISE_DHLEN);
	return false;
}

bool secure_link_enroll(const uint8_t station_public[NOISE_DHLEN],
			const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN])
{
	(void)station_public;
	(void)identity;
	PX4_ERR("no keystore on this board: enable the PX4 crypto backend");
	return false;
}

#endif /* PX4_CRYPTO */
