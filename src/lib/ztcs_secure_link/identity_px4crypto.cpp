/****************************************************************************
 * Identity key through the PX4 crypto backend. On an i.MX9 it is the P-256
 * key the EdgeLock Enclave generated and never exports, and on an SE05x board
 * the one the element holds, so no identity bytes are ever unwrapped from the
 * keystore. Elsewhere it is Ed25519 in a keystore slot, which only the kernel
 * reads.
 ****************************************************************************/

#include "secure_link_identity.h"
#include "secure_link_slots.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <string.h>

#if defined(PX4_CRYPTO) && !defined(ZTCS_IDENTITY_SE05X)

#include <px4_platform_common/crypto.h>

#if defined(CONFIG_ARCH_CHIP_IMX9) || defined(CONFIG_SSRC_CRYPTO_SE05X)

/* The enclave key, asked to hash the message itself. */
#define ELE_IDENTITY_MSG_INDEX 0xe1

bool secure_link_identity_public(struct secure_link_identity *id)
{
	PX4Crypto crypto;
	uint8_t xy[64];
	size_t len = sizeof(xy);
	bool ok = false;

	memset(id, 0, sizeof(*id));

	if (!crypto.open(CRYPTO_ECDSA_P256)) {
		PX4_ERR("no crypto session");
		return false;
	}

	/* The enclave generates the key on first ask. */
	if (crypto.get_public_key(ELE_IDENTITY_MSG_INDEX, xy, &len) && len == sizeof(xy)) {
		id->version = (xy[63] & 1) ? NOISE_PAYLOAD_VERSION_P256_ODD : NOISE_PAYLOAD_VERSION_P256_EVEN;
		memcpy(id->public_key, xy, sizeof(id->public_key));
		ok = true;

	} else {
		PX4_ERR("no identity key in the enclave");
	}

	crypto.close();
	return ok;
}

bool secure_link_identity_sign(const uint8_t *msg, size_t msg_len,
			       uint8_t sig[64])
{
	PX4Crypto crypto;

	memset(sig, 0, 64);

	if (!crypto.open(CRYPTO_ECDSA_P256)) {
		PX4_ERR("no crypto session");
		return false;
	}

	bool ok = crypto.sign(ELE_IDENTITY_MSG_INDEX, sig, msg, msg_len);
	crypto.close();

	if (!ok) {
		PX4_ERR("could not sign with the identity key");
		memset(sig, 0, 64);
	}

	return ok;
}

#else

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

#endif /* CONFIG_ARCH_CHIP_IMX9 || CONFIG_SSRC_CRYPTO_SE05X */

#endif /* PX4_CRYPTO && !ZTCS_IDENTITY_SE05X */
