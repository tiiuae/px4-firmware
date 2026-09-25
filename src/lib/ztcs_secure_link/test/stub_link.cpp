#include "secure_link.h"
#include <cstring>

int secure_link_init(struct secure_link *sl, const struct secure_link_keys *, uint64_t)
{
	std::memset(sl, 0, sizeof(*sl));
	sl->state = SECURE_LINK_HANDSHAKING;
	sl->rounds_to_establish = 2;
	return 0;
}

/* A handshake datagram is due while the link is coming up, and nothing once
 * it is, which is what the real one does.
 */
int secure_link_poll(struct secure_link *sl, uint64_t, uint8_t *out, size_t cap)
{
	sl->polls++;
	if (sl->state == SECURE_LINK_ESTABLISHED || cap < 4) { return 0; }
	std::memcpy(out, "HS\x00\x01", 4);
	return 4;
}

int secure_link_seal(struct secure_link *sl, uint64_t, const uint8_t *pt, size_t pt_len,
		     uint8_t *out, size_t cap)
{
	sl->seals++;
	if (sl->state != SECURE_LINK_ESTABLISHED) { return -1; }
	if (cap < pt_len + 4) { return -1; }
	std::memcpy(out, "PT", 2);
	std::memcpy(out + 2, pt, pt_len);
	return (int)pt_len + 2;
}

/* "HS" advances the handshake and is consumed; "PT" is payload. */
int secure_link_open(struct secure_link *sl, uint64_t, const uint8_t *frame, size_t len,
		     uint8_t *out, size_t cap)
{
	sl->opens++;
	if (len >= 2 && frame[0] == 'H' && frame[1] == 'S') {
		if (--sl->rounds_to_establish <= 0) {
			sl->state = SECURE_LINK_ESTABLISHED;
			sl->handshakes++;
		}
		return 0;
	}
	if (len >= 2 && frame[0] == 'P' && frame[1] == 'T') {
		size_t n = len - 2;
		if (n > cap) { return -1; }
		std::memcpy(out, frame + 2, n);
		return (int)n;
	}
	sl->decrypt_fails++;
	return -1;
}

bool secure_link_ensure_keys(struct secure_link_keys *) { return true; }
