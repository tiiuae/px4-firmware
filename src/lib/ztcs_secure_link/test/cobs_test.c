/****************************************************************************
 * COBS vectors, shared with ztcs-mavlink-gateway's cobs.rs.
 ****************************************************************************/

#include "cobs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

static void check(const char *name, int ok)
{
  printf("%s %s\n", ok ? "ok  " : "FAIL", name);
  failures += !ok;
}

static void vector(const char *name, const uint8_t *in, size_t len,
                   const uint8_t *want, size_t want_len)
{
  uint8_t enc[COBS_ENCODED_MAX(300)];
  uint8_t dec[300];
  size_t n = cobs_encode(in, len, enc);
  int m = cobs_decode(enc, n, dec, sizeof(dec));

  check(name, n == want_len && memcmp(enc, want, n) == 0 &&
        m == (int)len && memcmp(dec, in, len) == 0);
}

static void fill(uint8_t *out, size_t first, size_t count)
{
  for (size_t i = 0; i < count; i++)
    {
      out[i] = (uint8_t)(first + i);
    }
}

int main(void)
{
  uint8_t in[300];
  uint8_t want[300];
  struct cobs_rx rx = {0};
  uint8_t frame[64];
  uint8_t stream[] = {0, 3, 0x11, 0x22, 2, 0x33, 0, 0, 5, 0, 0, 2, 0x44, 0};
  int got[8];
  int n = 0;

  vector("00", (const uint8_t[]){0}, 1, (const uint8_t[]){1, 1}, 2);
  vector("00 00", (const uint8_t[]){0, 0}, 2, (const uint8_t[]){1, 1, 1}, 3);
  vector("00 11 00", (const uint8_t[]){0, 0x11, 0}, 3,
         (const uint8_t[]){1, 2, 0x11, 1}, 4);
  vector("11 22 00 33", (const uint8_t[]){0x11, 0x22, 0, 0x33}, 4,
         (const uint8_t[]){3, 0x11, 0x22, 2, 0x33}, 5);
  vector("11 22 33 44", (const uint8_t[]){0x11, 0x22, 0x33, 0x44}, 4,
         (const uint8_t[]){5, 0x11, 0x22, 0x33, 0x44}, 5);
  vector("11 00 00 00", (const uint8_t[]){0x11, 0, 0, 0}, 4,
         (const uint8_t[]){2, 0x11, 1, 1, 1}, 5);

  fill(in, 1, 254);
  want[0] = 0xff;
  fill(want + 1, 1, 254);
  vector("01..FE", in, 254, want, 255);

  in[0] = 0;
  fill(in + 1, 1, 254);
  want[0] = 1;
  want[1] = 0xff;
  fill(want + 2, 1, 254);
  vector("00 01..FE", in, 255, want, 256);

  fill(in, 1, 255);
  want[0] = 0xff;
  fill(want + 1, 1, 254);
  want[255] = 2;
  want[256] = 0xff;
  vector("01..FF", in, 255, want, 257);

  fill(in, 2, 254);
  in[254] = 0;
  want[0] = 0xff;
  fill(want + 1, 2, 254);
  want[255] = 1;
  want[256] = 1;
  vector("02..FF 00", in, 255, want, 257);

  fill(in, 3, 253);
  in[253] = 0;
  in[254] = 1;
  want[0] = 0xfe;
  fill(want + 1, 3, 253);
  want[254] = 2;
  want[255] = 1;
  vector("03..FF 00 01", in, 255, want, 256);

  for (size_t i = 0; i < sizeof(stream); i++)
    {
      int r = cobs_rx_push(&rx, stream[i], frame, sizeof(frame));

      if (r != 0 && n < 8)
        {
          got[n++] = r;
        }
    }

  check("stream: two frames, a bad one between", n == 3 && got[0] == 4 &&
        got[1] == -1 && got[2] == 1 && frame[0] == 0x44);

  memset(&rx, 0, sizeof(rx));

  for (size_t i = 0; i < COBS_FRAME_MAX + 10; i++)
    {
      cobs_rx_push(&rx, 1, frame, sizeof(frame));
    }

  check("overflow drops the frame", cobs_rx_push(&rx, 0, frame, sizeof(frame)) == -1);
  check("and recovers", cobs_rx_push(&rx, 2, frame, sizeof(frame)) == 0 &&
        cobs_rx_push(&rx, 0x44, frame, sizeof(frame)) == 0 &&
        cobs_rx_push(&rx, 0, frame, sizeof(frame)) == 1 && frame[0] == 0x44);

  printf("%s\n", failures ? "FAILED" : "all passed");
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
