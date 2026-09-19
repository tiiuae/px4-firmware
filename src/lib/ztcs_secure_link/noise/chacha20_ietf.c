/* ChaCha20 with the RFC 8439 nonce layout: 96-bit nonce, 32-bit block counter.
 *
 * Written out rather than taken from the OS because NuttX ships the DJB
 * variant, whose 64-bit nonce and separate salt word are not what Noise
 * specifies. Getting that wrong is silent and non-interoperable, so the one
 * primitive whose layout matters is kept here where it can be tested.
 */

#include "chacha20_ietf.h"

#include <string.h>

#define ROTL(a, b) (((a) << (b)) | ((a) >> (32 - (b))))

#define QR(a, b, c, d)                                                         \
  a += b, d ^= a, d = ROTL(d, 16), c += d, b ^= c, b = ROTL(b, 12), a += b,    \
  d ^= a, d = ROTL(d, 8), c += d, b ^= c, b = ROTL(b, 7)

static uint32_t load_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void store_le32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

void chacha20_ietf_block(const uint8_t key[32], const uint8_t nonce[12],
                         uint32_t counter, uint8_t out[64]) {
  static const char SIGMA[] = "expand 32-byte k";
  uint32_t s[16];
  uint32_t x[16];
  int i;

  for (i = 0; i < 4; i++) {
    s[i] = load_le32((const uint8_t *)SIGMA + 4 * i);
  }
  for (i = 0; i < 8; i++) {
    s[4 + i] = load_le32(key + 4 * i);
  }
  s[12] = counter;
  for (i = 0; i < 3; i++) {
    s[13 + i] = load_le32(nonce + 4 * i);
  }

  memcpy(x, s, sizeof(x));
  for (i = 0; i < 10; i++) {
    QR(x[0], x[4], x[8], x[12]);
    QR(x[1], x[5], x[9], x[13]);
    QR(x[2], x[6], x[10], x[14]);
    QR(x[3], x[7], x[11], x[15]);
    QR(x[0], x[5], x[10], x[15]);
    QR(x[1], x[6], x[11], x[12]);
    QR(x[2], x[7], x[8], x[13]);
    QR(x[3], x[4], x[9], x[14]);
  }
  for (i = 0; i < 16; i++) {
    store_le32(out + 4 * i, x[i] + s[i]);
  }
}

void chacha20_ietf_xor(const uint8_t key[32], const uint8_t nonce[12],
                       uint32_t counter, uint8_t *data, size_t len) {
  uint8_t block[64];
  size_t off = 0;

  while (off < len) {
    size_t n = len - off < 64 ? len - off : 64;
    size_t i;
    chacha20_ietf_block(key, nonce, counter, block);
    for (i = 0; i < n; i++) {
      data[off + i] ^= block[i];
    }
    off += n;
    counter++;
  }
  memset(block, 0, sizeof(block));
}
