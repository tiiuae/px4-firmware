#include "../cobs.h"
#include "../secure_link.h"

#include <stdint.h>
#include <string.h>

static uint8_t rng_state;

int noise_random(uint8_t *out, size_t len)
{
  for (size_t i = 0; i < len; i++)
    {
      out[i] = (uint8_t)(rng_state++ * 167u + 13u);
    }

  return 0;
}

static struct secure_link_keys fuzz_keys(void)
{
  struct secure_link_keys k;
  memset(&k, 0, sizeof(k));
  memset(k.link.sk, 0x11, sizeof(k.link.sk));
  memset(k.station_public, 0x22, sizeof(k.station_public));
  k.identity[0] = NOISE_PAYLOAD_VERSION;
  return k;
}

static void handshake_reply(const uint8_t *data, size_t size)
{
  struct secure_link sl;
  struct secure_link_keys k = fuzz_keys();
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;

  if (secure_link_init(&sl, &k, t) < 0)
    {
      return;
    }

  secure_link_poll(&sl, t, out, sizeof(out));
  secure_link_open(&sl, t + 1000, data, size, out, sizeof(out));
  secure_link_close(&sl);
}

static void established(const uint8_t *data, size_t size)
{
  struct secure_link sl;
  struct secure_link_keys k = fuzz_keys();
  uint8_t key_a[NOISE_KEYLEN];
  uint8_t key_b[NOISE_KEYLEN];
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;

  if (secure_link_init(&sl, &k, t) < 0)
    {
      return;
    }

  memset(&sl.session, 0, sizeof(sl.session));
  memset(key_a, 0xa5, sizeof(key_a));
  memset(key_b, 0x5a, sizeof(key_b));
  noise_session_key_set(&sl.session.send, key_a);
  noise_session_key_set(&sl.session.recv, key_b);
  sl.state = SECURE_LINK_ESTABLISHED;
  sl.last_open_us = t;
  sl.established_us = t;
  sl.next_rekey_us = t + SECURE_LINK_REKEY_US;

  while (size >= 2)
    {
      size_t n = data[0] % (size - 1) + 1;
      secure_link_open(&sl, t, data + 1, n, out, sizeof(out));
      data += n + 1;
      size -= n + 1;
      t += 1000;
    }

  secure_link_close(&sl);
}

static void cobs(const uint8_t *data, size_t size)
{
  static struct cobs_rx rx;
  uint8_t out[COBS_FRAME_MAX];

  cobs_decode(data, size, out, sizeof(out));
  cobs_decode(data, size, out, size / 2 < sizeof(out) ? size / 2 : sizeof(out));
  memset(&rx, 0, sizeof(rx));

  for (size_t i = 0; i < size; i++)
    {
      cobs_rx_push(&rx, data[i], out, sizeof(out));
    }
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
  if (size < 1)
    {
      return 0;
    }

  rng_state = 0;

  switch (data[0] % 3)
    {
    case 0:
      handshake_reply(data + 1, size - 1);
      break;
    case 1:
      established(data + 1, size - 1);
      break;
    default:
      cobs(data + 1, size - 1);
      break;
    }

  return 0;
}
