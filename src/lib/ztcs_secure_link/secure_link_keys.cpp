/****************************************************************************
 * Key material for the link.
 *
 * No private key appears in this address space. The link key is a keystore
 * slot the kernel performs the exchange with, and the identity key only ever
 * signs. Both are generated on the aircraft and neither can be read back.
 ****************************************************************************/

#include "secure_link.h"
#include "secure_link_identity.h"
#include "secure_link_slots.h"

#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#if defined(PX4_CRYPTO)

#include <px4_platform_common/crypto.h>

/* The signed identity payload is public and fixed once the link key is: it
 * is signed at enrolment and read back, so the identity key signs once per
 * device, never at boot.
 */
static const char identity_path[] = ZTCS_IDENTITY_PAYLOAD_PATH;

struct stored_identity {
	uint8_t link_public[NOISE_DHLEN];
	uint8_t payload[NOISE_IDENTITY_PAYLOAD_LEN];
};

/* True only if the file is for this link key and this identity key. */
static bool load_identity(uint8_t out[NOISE_IDENTITY_PAYLOAD_LEN])
{
	struct stored_identity st;
	struct secure_link_identity id;
	uint8_t link_public[NOISE_DHLEN];
	int fd = open(identity_path, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	bool ok = read(fd, &st, sizeof(st)) == (ssize_t)sizeof(st);
	close(fd);

	ok = ok && secure_link_public_key(link_public)
	     && memcmp(st.link_public, link_public, NOISE_DHLEN) == 0
	     && secure_link_identity_public(&id)
	     && st.payload[0] == id.version
	     && memcmp(st.payload + 1, id.public_key, sizeof(id.public_key)) == 0;

	if (ok) {
		memcpy(out, st.payload, NOISE_IDENTITY_PAYLOAD_LEN);
	}

	return ok;
}

static bool store_identity(const uint8_t link_public[NOISE_DHLEN],
			   const uint8_t payload[NOISE_IDENTITY_PAYLOAD_LEN])
{
	struct stored_identity st;
	int fd = open(identity_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (fd < 0) {
		return false;
	}

	memcpy(st.link_public, link_public, NOISE_DHLEN);
	memcpy(st.payload, payload, NOISE_IDENTITY_PAYLOAD_LEN);
	bool ok = write(fd, &st, sizeof(st)) == (ssize_t)sizeof(st) && fsync(fd) == 0;
	close(fd);
	return ok;
}

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

	if (!load_identity(keys->identity)) {
		PX4_WARN("no signed identity for this link key: run ztcs_enroll sign");
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

bool secure_link_self_sign(uint8_t out[NOISE_IDENTITY_PAYLOAD_LEN])
{
	struct secure_link_identity id;
	uint8_t link_public[NOISE_DHLEN];
	uint8_t signed_input[sizeof(NOISE_STATIC_KEY_CONTEXT) - 1 + NOISE_DHLEN];

	if (load_identity(out)) {
		return true;
	}

	memset(out, 0, NOISE_IDENTITY_PAYLOAD_LEN);

	if (!secure_link_public_key(link_public)) {
		PX4_ERR("no link key to sign");
		return false;
	}

	if (!secure_link_identity_public(&id)) {
		return false;
	}

	out[0] = id.version;
	memcpy(out + 1, id.public_key, sizeof(id.public_key));

	noise_static_key_signing_input(link_public, signed_input);

	if (!secure_link_identity_sign(signed_input, sizeof(signed_input),
				       out + 1 + sizeof(id.public_key))) {
		PX4_ERR("could not sign the link key");
		memset(out, 0, NOISE_IDENTITY_PAYLOAD_LEN);
		return false;
	}

	if (!store_identity(link_public, out)) {
		PX4_ERR("could not store the signed identity");
		memset(out, 0, NOISE_IDENTITY_PAYLOAD_LEN);
		return false;
	}

	return true;
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
