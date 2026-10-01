/****************************************************************************
 * COBS framing for the secure link over a byte stream.
 ****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COBS_ENCODED_MAX(n) ((n) + (n) / 254 + 1)
#define COBS_FRAME_MAX      520

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
