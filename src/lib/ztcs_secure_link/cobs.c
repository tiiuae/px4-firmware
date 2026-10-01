#include "cobs.h"

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
  size_t code_at = 0;
  size_t o = 1;
  uint8_t code = 1;

  for (size_t i = 0; i < len; i++)
    {
      if (in[i] == 0)
        {
          out[code_at] = code;
          code_at = o++;
          code = 1;
        }
      else
        {
          out[o++] = in[i];

          if (++code == 0xff && i + 1 < len)
            {
              out[code_at] = code;
              code_at = o++;
              code = 1;
            }
        }
    }

  out[code_at] = code;
  return o;
}

int cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t cap)
{
  size_t i = 0;
  size_t o = 0;

  while (i < len)
    {
      uint8_t code = in[i++];

      if (code == 0 || i + code - 1 > len)
        {
          return -1;
        }

      for (uint8_t k = 1; k < code; k++)
        {
          if (o == cap || in[i] == 0)
            {
              return -1;
            }

          out[o++] = in[i++];
        }

      if (code != 0xff && i < len)
        {
          if (o == cap)
            {
              return -1;
            }

          out[o++] = 0;
        }
    }

  return (int)o;
}

int cobs_rx_push(struct cobs_rx *rx, uint8_t byte, uint8_t *out, size_t cap)
{
  int n;

  if (byte != 0)
    {
      if (rx->len < sizeof(rx->buf))
        {
          rx->buf[rx->len++] = byte;
        }
      else
        {
          rx->overflow = true;
        }

      return 0;
    }

  if (rx->len == 0 && !rx->overflow)
    {
      return 0;
    }

  n = rx->overflow ? -1 : cobs_decode(rx->buf, rx->len, out, cap);
  rx->len = 0;
  rx->overflow = false;
  return n;
}
