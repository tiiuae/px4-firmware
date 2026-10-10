#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COBS_ENCODED_MAX(n) ((n) + (n) / 254 + 1)

/* Big enough for the largest datagram the link can carry, which is a
 * handshake message rather than a transport frame once the KEM fields are in
 * it. The receive buffer is one of these per link.
 */
#ifdef NOISE_HFS
#define COBS_FRAME_MAX      1536
#else
#define COBS_FRAME_MAX      520
#endif

struct cobs_rx
{
  uint8_t buf[COBS_FRAME_MAX];
  size_t len;
  bool overflow;
};

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out);

int cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t cap);

int cobs_rx_push(struct cobs_rx *rx, uint8_t byte, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif
