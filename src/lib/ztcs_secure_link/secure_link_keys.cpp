/****************************************************************************
 * Key material for the link.
 *
 * No private key appears in this address space. The link key is a keystore
 * slot the kernel performs the exchange with, and the identity key only ever
 * signs. Both are generated on the aircraft and neither can be read back.
 ****************************************************************************/

#include "secure_link.h"

#include <px4_platform_common/log.h>

#include <string.h>

#if defined(PX4_CRYPTO)

#include <px4_platform_common/crypto.h>

#ifndef ZTCS_KEY_SLOT_STATION_PUBLIC
#define ZTCS_KEY_SLOT_STATION_PUBLIC 3
#endif
#ifndef ZTCS_KEY_SLOT_LINK
#define ZTCS_KEY_SLOT_LINK 15
#endif
#ifndef ZTCS_KEY_SLOT_IDENTITY
#define ZTCS_KEY_SLOT_IDENTITY 17
#endif
#ifndef ZTCS_KEY_SLOT_OPERATOR_PUBLIC
#define ZTCS_KEY_SLOT_OPERATOR_PUBLIC 4
#endif

static bool operator_pinned(PX4Crypto &crypto, uint8_t out[32])
{
	size_t len = 32;
	return crypto.get_public_key(ZTCS_KEY_SLOT_OPERATOR_PUBLIC, out, &len) && len == 32;
}

bool secure_link_ensure_keys(struct secure_link_keys *keys)
{
	PX4Crypto crypto;
	size_t len = NOISE_DHLEN;
	uint8_t probe[NOISE_DHLEN];

	memset(keys, 0, sizeof(*keys));
	keys->link.index = ZTCS_KEY_SLOT_LINK;

	if (!crypto.open(CRYPTO_X25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	if (!crypto.get_public_key(ZTCS_KEY_SLOT_LINK, probe, &len)) {
		if (!crypto.generate_key(ZTCS_KEY_SLOT_LINK, true)) {
			PX4_ERR("could not establish a link key");
			crypto.close();
			return false;
		}

		PX4_INFO("link key generated");
	}

	len = NOISE_DHLEN;

	if (!crypto.get_public_key(ZTCS_KEY_SLOT_STATION_PUBLIC, keys->station_public, &len)
	    || len != NOISE_DHLEN) {
		PX4_WARN("not enrolled yet: no station key");
		crypto.close();
		memset(keys, 0, sizeof(*keys));
		return false;
	}

	crypto.close();

	if (!secure_link_self_sign(keys->identity)) {
		memset(keys, 0, sizeof(*keys));
		return false;
	}

	return true;
}

bool secure_link_public_key(uint8_t out[NOISE_DHLEN])
{
	struct noise_static_key s;

	s.index = ZTCS_KEY_SLOT_LINK;
	return noise_static_public(&s, out) == 0;
}

bool secure_link_pin_operator(const uint8_t operator_public[32])
{
	PX4Crypto crypto;
	uint8_t existing[32];

	if (!crypto.open(CRYPTO_ED25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	if (operator_pinned(crypto, existing)) {
		crypto.close();
		PX4_ERR("an operator key is already pinned");
		return false;
	}

	bool ok = crypto.set_key(0, nullptr, operator_public, 32,
				 ZTCS_KEY_SLOT_OPERATOR_PUBLIC);
	crypto.close();

	if (!ok) {
		PX4_ERR("could not pin the operator key");
	}

	return ok;
}

bool secure_link_enroll(const uint8_t station_public[NOISE_DHLEN],
			const uint8_t *signature)
{
	PX4Crypto crypto;
	uint8_t op[32];
	bool ok;

	/* The session algorithm is the one the signature is checked under, not
	 * the one being stored.
	 */
	if (!crypto.open(CRYPTO_ED25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	if (operator_pinned(crypto, op)) {
		if (signature == NULL) {
			crypto.close();
			PX4_ERR("an operator key is pinned: this write must be signed");
			return false;
		}

		ok = crypto.set_key(ZTCS_KEY_SLOT_OPERATOR_PUBLIC, signature, station_public,
				    NOISE_DHLEN, ZTCS_KEY_SLOT_STATION_PUBLIC);

	} else {
		ok = crypto.set_key(0, nullptr, station_public, NOISE_DHLEN,
				    ZTCS_KEY_SLOT_STATION_PUBLIC);
	}

	crypto.close();

	if (!ok) {
		PX4_ERR("could not store the station key");
	}

	return ok;
}

/* Ed25519 here is the RFC 8032 construction over SHA-512, which is what the
 * ground station verifies with.
 */
bool secure_link_self_sign(uint8_t out[NOISE_IDENTITY_PAYLOAD_LEN])
{
	PX4Crypto crypto;
	uint8_t link_public[NOISE_DHLEN];
	uint8_t signed_input[sizeof(NOISE_STATIC_KEY_CONTEXT) - 1 + NOISE_DHLEN];
	size_t len = 32;
	bool ok = false;

	memset(out, 0, NOISE_IDENTITY_PAYLOAD_LEN);

	if (!secure_link_public_key(link_public)) {
		PX4_ERR("no link key to sign");
		return false;
	}

	if (!crypto.open(CRYPTO_ED25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	if (!crypto.get_public_key(ZTCS_KEY_SLOT_IDENTITY, out + 1, &len)) {
		if (!crypto.generate_key(ZTCS_KEY_SLOT_IDENTITY, true)) {
			PX4_ERR("could not establish an identity key");
			goto out_close;
		}

		PX4_INFO("identity key generated");
		len = 32;

		if (!crypto.get_public_key(ZTCS_KEY_SLOT_IDENTITY, out + 1, &len)) {
			goto out_close;
		}
	}

	if (len != 32) {
		PX4_ERR("identity key is %d bytes, want 32", (int)len);
		goto out_close;
	}

	out[0] = NOISE_PAYLOAD_VERSION;
	noise_static_key_signing_input(link_public, signed_input);
	ok = crypto.sign(ZTCS_KEY_SLOT_IDENTITY, out + 1 + 32, signed_input,
			 sizeof(signed_input));

	if (!ok) {
		PX4_ERR("could not sign the link key");
	}

out_close:
	crypto.close();

	if (!ok) {
		memset(out, 0, NOISE_IDENTITY_PAYLOAD_LEN);
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

bool secure_link_pin_operator(const uint8_t operator_public[32])
{
	(void)operator_public;
	PX4_ERR("no keystore on this board: enable the PX4 crypto backend");
	return false;
}

bool secure_link_enroll(const uint8_t station_public[NOISE_DHLEN],
			const uint8_t *signature)
{
	(void)station_public;
	(void)signature;
	PX4_ERR("no keystore on this board: enable the PX4 crypto backend");
	return false;
}

bool secure_link_self_sign(uint8_t out[NOISE_IDENTITY_PAYLOAD_LEN])
{
	memset(out, 0, NOISE_IDENTITY_PAYLOAD_LEN);
	PX4_ERR("no keystore on this board: enable the PX4 crypto backend");
	return false;
}

#endif /* PX4_CRYPTO */
