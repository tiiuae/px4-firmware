/****************************************************************************
 * Identity key in the PX4 keystore: Ed25519, for a part whose enclave holds
 * the private half where this address space cannot read it.
 *
 * The i.MX93 ELE is the case this was written for. A part without one uses
 * the secure element backend instead.
 ****************************************************************************/

#include "secure_link_identity.h"
#include "secure_link_slots.h"

#include <px4_platform_common/log.h>

#include <string.h>

#if defined(PX4_CRYPTO) && !defined(ZTCS_IDENTITY_SE05X)

#include <px4_platform_common/crypto.h>

bool secure_link_identity_public(struct secure_link_identity *id)
{
	PX4Crypto crypto;
	size_t len = sizeof(id->public_key);
	bool ok = false;

	memset(id, 0, sizeof(*id));
	id->version = NOISE_PAYLOAD_VERSION;

	if (!crypto.open(CRYPTO_ED25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	if (!crypto.get_public_key(ZTCS_KEY_SLOT_IDENTITY, id->public_key, &len)) {
		if (!crypto.generate_key(ZTCS_KEY_SLOT_IDENTITY, true)) {
			PX4_ERR("could not establish an identity key");
			goto out_close;
		}

		PX4_INFO("identity key generated");
		len = sizeof(id->public_key);

		if (!crypto.get_public_key(ZTCS_KEY_SLOT_IDENTITY, id->public_key, &len)) {
			goto out_close;
		}
	}

	if (len != sizeof(id->public_key)) {
		PX4_ERR("identity key is %d bytes, want 32", (int)len);
		goto out_close;
	}

	ok = true;

out_close:
	crypto.close();

	if (!ok) {
		memset(id, 0, sizeof(*id));
	}

	return ok;
}

/* RFC 8032 over SHA-512, which is what the ground station verifies with.
 * PX4's monocypher also exports a BLAKE2b variant that does not interoperate.
 */
bool secure_link_identity_sign(const uint8_t *msg, size_t msg_len,
			       uint8_t sig[64])
{
	PX4Crypto crypto;

	memset(sig, 0, 64);

	if (!crypto.open(CRYPTO_ED25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	bool ok = crypto.sign(ZTCS_KEY_SLOT_IDENTITY, sig, msg, msg_len);
	crypto.close();

	if (!ok) {
		PX4_ERR("could not sign with the identity key");
		memset(sig, 0, 64);
	}

	return ok;
}

#endif /* PX4_CRYPTO && !ZTCS_IDENTITY_SE05X */
