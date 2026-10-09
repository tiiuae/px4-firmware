/****************************************************************************
 * Secure link, aircraft side. See secure_link.h.
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

static uint64_t since(uint64_t now_us, uint64_t then_us)
{
  return now_us > then_us ? now_us - then_us : 0;
}

static int start_handshake(struct secure_link *sl, uint64_t now_us,
                           uint8_t *out, size_t cap)
{
  size_t n = 0;
  int rc;

  sl->state = SECURE_LINK_HANDSHAKING;
  sl->decrypt_fails = 0;
  sl->next_retry_us = now_us + sl->retry_interval_us
                      + jitter_us(sl->retry_interval_us);

  if (cap < NOISE_MSG1_LEN)
    {
      return NOISE_ERR_INPUT;
    }

  rc = noise_hs_start(&sl->hs, &sl->keys.link,
                      sl->keys.station_public, sl->keys.identity,
                      out, &n);
  if (rc != NOISE_OK)
    {
      return rc;
    }

  sl->handshakes++;
  return (int)n;
}

static void end_session(struct secure_link *sl)
{
  noise_session_wipe(&sl->session);
  noise_session_wipe(&sl->previous);
  sl->has_previous = false;
  sl->rekeying = false;
}

static int send_rekey(struct secure_link *sl, uint8_t *out, size_t cap)
{
  size_t n = 0;
  int rc;

  if (cap < NOISE_MSG1_LEN)
    {
      return NOISE_ERR_INPUT;
    }

  rc = noise_hs_start(&sl->hs, &sl->keys.link,
                      sl->keys.station_public, sl->keys.identity,
                      out, &n);
  if (rc != NOISE_OK)
    {
      return rc;
    }

  sl->rekeying = true;
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
      if (since(now_us, sl->last_open_us) >= SECURE_LINK_SILENCE_US
          || since(now_us, sl->established_us) >= SECURE_LINK_MAX_AGE_US)
        {
          if (since(now_us, sl->last_open_us) >= SECURE_LINK_SILENCE_US)
            {
              sl->silence_drops++;
              sl->last_drop_us = now_us;
            }
          else
            {
              sl->age_drops++;
              sl->last_drop_us = now_us;
            }

          end_session(sl);
          reset_backoff(sl);
          return start_handshake(sl, now_us, out, cap);
        }

      if (now_us < sl->next_rekey_us)
        {
          return 0;
        }

      {
        int n = send_rekey(sl, out, cap);
        sl->next_rekey_us = now_us + sl->retry_interval_us;
        bump_backoff(sl);
        return n;
      }
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
      end_session(sl);
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
        {
          struct noise_session next;

          if (sl->state != SECURE_LINK_HANDSHAKING && !sl->rekeying)
            {
              return NOISE_ERR_STATE;
            }

          memset(&next, 0, sizeof(next));
          rc = noise_hs_finish(&sl->hs, frame, len, &next);
          if (rc != NOISE_OK)
            {
              return rc;
            }

          noise_session_wipe(&sl->previous);
          sl->has_previous = sl->state == SECURE_LINK_ESTABLISHED;
          if (sl->has_previous)
            {
              sl->previous = sl->session;
            }

          sl->session = next;
          noise_wipe(&next, sizeof(next));
          sl->state = SECURE_LINK_ESTABLISHED;
          sl->rekeying = false;
          sl->decrypt_fails = 0;
          sl->last_open_us = now_us;
          sl->established_us = now_us;
          sl->next_rekey_us = now_us + SECURE_LINK_REKEY_US
                              + jitter_us(SECURE_LINK_REKEY_US);
          reset_backoff(sl);
          return 0;
        }

      case NOISE_TYPE_TRANSPORT:
        if (sl->state != SECURE_LINK_ESTABLISHED)
          {
            return NOISE_ERR_STATE;
          }

        if (len < NOISE_TRANSPORT_HDR_LEN + NOISE_TAGLEN
            || cap < len - NOISE_TRANSPORT_HDR_LEN - NOISE_TAGLEN)
          {
            return NOISE_ERR_INPUT;
          }

        rc = noise_session_open(&sl->session, frame, len, out, &n);
        if (rc == NOISE_OK && sl->has_previous)
          {
            noise_session_wipe(&sl->previous);
            sl->has_previous = false;
          }
        else if (rc == NOISE_ERR_DECRYPT && sl->has_previous)
          {
            rc = noise_session_open(&sl->previous, frame, len, out, &n);
          }

        if (rc != NOISE_OK)
          {
            if (rc == NOISE_ERR_REPLAY)
              {
                sl->replays++;
              }

            /* A replay is a radio event, not evidence of a rekey. */
            if (rc != NOISE_ERR_REPLAY
                && ++sl->decrypt_fails >= SECURE_LINK_DECRYPT_FAILS)
              {
                sl->decrypt_fails = 0;

                if (!sl->rekeying
                    && (sl->fail_rekeys == 0
                        || since(now_us, sl->last_fail_rekey_us)
                           >= SECURE_LINK_SILENCE_US))
                  {
                    sl->fail_rekeys++;
                    sl->last_fail_rekey_us = now_us;
                    sl->next_rekey_us = now_us;
                  }
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

void secure_link_close(struct secure_link *sl)
{
  end_session(sl);
  noise_hs_end(&sl->hs);
  sl->state = SECURE_LINK_DOWN;
}
