#pragma once
#include <cstdint>
#include <cstddef>
#define NOISE_DHLEN 32
#define NOISE_TAGLEN 16
#define NOISE_TRANSPORT_HDR_LEN 9
#define NOISE_IDENTITY_PAYLOAD_LEN 97
enum secure_link_state { SECURE_LINK_INIT = 0, SECURE_LINK_HANDSHAKING, SECURE_LINK_ESTABLISHED };
struct secure_link_keys { int dummy; };
struct secure_link {
	enum secure_link_state state;
	uint32_t handshakes;
	uint32_t decrypt_fails;
	/* test knobs */
	int rounds_to_establish;
	int polls;
	int seals;
	int opens;
};
extern "C" {
int secure_link_init(struct secure_link *sl, const struct secure_link_keys *k, uint64_t now);
int secure_link_poll(struct secure_link *sl, uint64_t now, uint8_t *out, size_t cap);
int secure_link_seal(struct secure_link *sl, uint64_t now, const uint8_t *pt, size_t pt_len, uint8_t *out, size_t cap);
int secure_link_open(struct secure_link *sl, uint64_t now, const uint8_t *frame, size_t len, uint8_t *out, size_t cap);
bool secure_link_ensure_keys(struct secure_link_keys *keys);
}
