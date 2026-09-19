/****************************************************************************
 * Secure MAVLink link, aircraft side. See secure_link.h.
 ****************************************************************************/

#include "secure_link.h"

#include <string.h>

/* So a fleet that lost the station together does not come back together. */
static uint64_t jitter_us(uint64_t interval_us)
{
  uint8_t r = 0;

  if (noise_random(&r, 1) != 0)
    {
      return 0;
    }

  return (interval_us / 4) * r / 255;
}

static int start_handshake(struct secure_link *sl, uint64_t now_us,
                           uint8_t *out, size_t cap)
{
  size_t n = 0;
  int rc;

  if (cap < NOISE_MSG1_LEN)
    {
      return NOISE_ERR_INPUT;
    }

  rc = noise_initiator_start(&sl->ini, sl->keys.static_private,
                             sl->keys.station_public, sl->keys.identity,
                             out, &n);
  if (rc != NOISE_OK)
    {
      return rc;
    }

  sl->state = SECURE_LINK_HANDSHAKING;
  sl->decrypt_fails = 0;
  sl->next_retry_us = now_us + sl->retry_interval_us
                      + jitter_us(sl->retry_interval_us);
  sl->handshakes++;
  return (int)n;
}

/* A fresh sequence starts at the bottom; a retransmit does not. */
static void reset_backoff(struct secure_link *sl)
{
  sl->retry_interval_us = SECURE_LINK_RETRY_MIN_US;
}

static void bump_backoff(struct secure_link *sl)
{
  if (sl->retry_interval_us >= SECURE_LINK_RETRY_CAP_US)
    {
      sl->retry_interval_us = SECURE_LINK_RETRY_SLOW_US;
      return;
    }

  sl->retry_interval_us *= 2;

  if (sl->retry_interval_us > SECURE_LINK_RETRY_CAP_US)
    {
      sl->retry_interval_us = SECURE_LINK_RETRY_CAP_US;
    }
}

int secure_link_init(struct secure_link *sl,
                     const struct secure_link_keys *keys, uint64_t now_us)
{
  memset(sl, 0, sizeof(*sl));
  sl->keys = *keys;
  sl->state = SECURE_LINK_HANDSHAKING;
  sl->last_open_us = now_us;

  /* Due immediately; poll() emits it so init does no I/O of its own. */
  reset_backoff(sl);
  sl->next_retry_us = now_us;
  return NOISE_OK;
}

int secure_link_poll(struct secure_link *sl, uint64_t now_us,
                     uint8_t *out, size_t cap)
{
  if (sl->state == SECURE_LINK_DOWN)
    {
      return 0;
    }

  if (sl->state == SECURE_LINK_ESTABLISHED)
    {
      /* Link gone, or peer stopped answering. Either way, drop it. */
      if (now_us - sl->last_open_us < SECURE_LINK_SILENCE_US)
        {
          return 0;
        }

      noise_session_wipe(&sl->session);
      reset_backoff(sl);
      return start_handshake(sl, now_us, out, cap);
    }

  if (now_us < sl->next_retry_us)
    {
      return 0;
    }

  {
    /* Schedule on the current interval, then widen: an absent station
     * costs a datagram every few seconds, not a flood.
     */
    int n = start_handshake(sl, now_us, out, cap);
    bump_backoff(sl);
    return n;
  }
}

int secure_link_seal(struct secure_link *sl, uint64_t now_us,
                     const uint8_t *pt, size_t pt_len,
                     uint8_t *out, size_t cap)
{
  size_t n = 0;
  int rc;

  (void)now_us;

  if (sl->state != SECURE_LINK_ESTABLISHED)
    {
      return NOISE_ERR_STATE;
    }

  if (cap < NOISE_TRANSPORT_HDR_LEN + pt_len + NOISE_TAGLEN)
    {
      return NOISE_ERR_INPUT;
    }

  rc = noise_session_seal(&sl->session, pt, pt_len, out, &n);
  if (rc == NOISE_ERR_EXHAUSTED)
    {
      /* The nonce must never wrap. poll() opens the next session. */
      noise_session_wipe(&sl->session);
      sl->state = SECURE_LINK_HANDSHAKING;
      reset_backoff(sl);
      sl->next_retry_us = 0;
      return rc;
    }

  return rc == NOISE_OK ? (int)n : rc;
}

int secure_link_open(struct secure_link *sl, uint64_t now_us,
                     const uint8_t *frame, size_t len,
                     uint8_t *out, size_t cap)
{
  size_t n = 0;
  int rc;

  if (len < 1)
    {
      return NOISE_ERR_INPUT;
    }

  switch (frame[0])
    {
      case NOISE_TYPE_HANDSHAKE_RESP:
        if (sl->state != SECURE_LINK_HANDSHAKING)
          {
            return NOISE_ERR_STATE;
          }

        rc = noise_initiator_finish(&sl->ini, frame, len, &sl->session);
        if (rc != NOISE_OK)
          {
            return rc;
          }

        sl->state = SECURE_LINK_ESTABLISHED;
        sl->decrypt_fails = 0;
        sl->last_open_us = now_us;
        return 0;

      case NOISE_TYPE_TRANSPORT:
        if (sl->state != SECURE_LINK_ESTABLISHED)
          {
            return NOISE_ERR_STATE;
          }

        if (cap < len)
          {
            return NOISE_ERR_INPUT;
          }

        rc = noise_session_open(&sl->session, frame, len, out, &n);
        if (rc != NOISE_OK)
          {
            /* A replay is a radio event, not evidence of a rekey. */
            if (rc != NOISE_ERR_REPLAY
                && ++sl->decrypt_fails >= SECURE_LINK_DECRYPT_FAILS)
              {
                noise_session_wipe(&sl->session);
                sl->state = SECURE_LINK_HANDSHAKING;
                reset_backoff(sl);
                sl->next_retry_us = 0;
              }

            return rc;
          }

        sl->decrypt_fails = 0;
        sl->last_open_us = now_us;
        return (int)n;

      default:
        /* An initiation aimed at an initiator, or noise on the port. */
        return NOISE_ERR_INPUT;
    }
}
