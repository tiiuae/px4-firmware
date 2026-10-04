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

static const char *reported;

static bool not_ready(struct secure_link_keys *keys, const char *reason, bool fault = false)
{
	if (reason != reported && fault) {
		PX4_ERR("%s", reason);

	} else if (reason != reported) {
		PX4_WARN("%s", reason);
	}

	reported = reason;

	memset(keys, 0, sizeof(*keys));
	return false;
}

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
		return not_ready(keys, "no crypto session", true);
	}

	if (!crypto.get_public_key(ZTCS_KEY_SLOT_LINK, probe, &len)) {
		if (!crypto.generate_key(ZTCS_KEY_SLOT_LINK, true)) {
			crypto.close();
			return not_ready(keys, "could not establish a link key", true);
		}

		PX4_INFO("link key generated");
	}

	len = NOISE_DHLEN;

	if (!crypto.get_public_key(ZTCS_KEY_SLOT_STATION_PUBLIC, keys->station_public, &len)
	    || len != NOISE_DHLEN) {
		crypto.close();
		return not_ready(keys, "not enrolled yet: no station key");
	}

	crypto.close();

	if (!load_identity(keys->identity)) {
		return not_ready(keys, "no signed identity for this link key: run ztcs_enroll sign");
	}

	reported = nullptr;
	return true;
}

bool secure_link_public_key(uint8_t out[NOISE_DHLEN])
{
	struct noise_static_key s;

	s.index = ZTCS_KEY_SLOT_LINK;
	return noise_static_public(&s, out) == 0;
}

bool secure_link_enrolment_closed(void)
{
	PX4Crypto ed;
	PX4Crypto x;
	uint8_t key[32];
	size_t len = NOISE_DHLEN;
	bool closed = true;

	if (ed.open(CRYPTO_ED25519) && x.open(CRYPTO_X25519)) {
		closed = operator_pinned(ed, key)
			 && x.get_public_key(ZTCS_KEY_SLOT_STATION_PUBLIC, key, &len)
			 && len == NOISE_DHLEN;
	}

	ed.close();
	x.close();
	return closed;
}

bool secure_link_enrolled(void)
{
	PX4Crypto ed;
	PX4Crypto x;
	uint8_t key[32];
	size_t len = NOISE_DHLEN;
	bool enrolled = ed.open(CRYPTO_ED25519) && x.open(CRYPTO_X25519)
			&& operator_pinned(ed, key)
			&& x.get_public_key(ZTCS_KEY_SLOT_STATION_PUBLIC, key, &len)
			&& len == NOISE_DHLEN;

	ed.close();
	x.close();
	return enrolled;
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
		const bool same = memcmp(existing, operator_public, sizeof(existing)) == 0;

		if (!same) {
			PX4_ERR("another operator key is already pinned");
		}

		return same;
	}

	bool ok = crypto.set_key(0, nullptr, operator_public, 32,
				 ZTCS_KEY_SLOT_OPERATOR_PUBLIC);
	crypto.close();

	if (!ok) {
		PX4_ERR("could not pin the operator key");
	}

	return ok;
}

static bool station_held(char hex[NOISE_DHLEN * 2 + 1])
{
	PX4Crypto crypto;
	uint8_t key[NOISE_DHLEN];
	size_t len = sizeof(key);
	bool held = crypto.open(CRYPTO_X25519) &&
		    crypto.get_public_key(ZTCS_KEY_SLOT_STATION_PUBLIC, key, &len) && len == sizeof(key);

	crypto.close();

	for (unsigned i = 0; held && i < sizeof(key); i++) {
		snprintf(&hex[i * 2], 3, "%02x", key[i]);
	}

	return held;
}

bool secure_link_enroll(const uint8_t station_public[NOISE_DHLEN],
			const uint8_t *signature)
{
	PX4Crypto crypto;
	uint8_t op[32];
	char held[NOISE_DHLEN * 2 + 1];
	bool pinned;
	bool ok;

	/* The session algorithm is the one the signature is checked under, not
	 * the one being stored.
	 */
	if (!crypto.open(CRYPTO_ED25519)) {
		PX4_ERR("no crypto session");
		return false;
	}

	pinned = operator_pinned(crypto, op);

	if (pinned) {
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

	if (ok) {
		return true;
	}

	const bool stored = station_held(held);

	if (pinned && stored) {
		PX4_ERR("signature refused: re-sign with --previous-station-public, held:");
		PX4_ERR("%s", held);

	} else if (pinned) {
		PX4_ERR("signature refused: not the pinned operator's, or not for this link key");

	} else if (stored) {
		PX4_ERR("a station key is held and no operator is pinned: only a reprovision replaces it");

	} else {
		PX4_ERR("could not store the station key");
	}

	return false;
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
	return not_ready(keys, "no keystore on this board: enable the PX4 crypto backend", true);
}

bool secure_link_public_key(uint8_t out[NOISE_DHLEN])
{
	memset(out, 0, NOISE_DHLEN);
	return false;
}

bool secure_link_enrolment_closed(void)
{
	return true;
}

bool secure_link_enrolled(void)
{
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
