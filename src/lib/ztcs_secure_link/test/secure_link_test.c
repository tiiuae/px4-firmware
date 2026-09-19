/****************************************************************************
 * Unit tests for the session state machine. No sockets and no real clock:
 * the library is sans-io precisely so a flight controller's timing can be
 * driven rather than waited for.
 ****************************************************************************/

#include "../secure_link.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond, ...)                                                   \
  do {                                                                     \
    if (!(cond)) {                                                         \
      printf("FAIL %s:%d ", __func__, __LINE__);                           \
      printf(__VA_ARGS__);                                                 \
      printf("\n");                                                        \
      failures++;                                                          \
    }                                                                      \
  } while (0)

static struct secure_link_keys dummy_keys(void)
{
  struct secure_link_keys k;
  memset(&k, 0, sizeof(k));
  memset(k.static_private, 0x11, sizeof(k.static_private));
  memset(k.station_public, 0x22, sizeof(k.station_public));
  k.identity[0] = NOISE_PAYLOAD_VERSION;
  return k;
}

/* Puts the link in ESTABLISHED with a session that can talk to `peer`, so
 * the timer paths can be exercised without a responder.
 */
static void fake_session(struct secure_link *sl, struct noise_session *peer,
                         uint64_t now_us)
{
  struct secure_link_keys k = dummy_keys();

  secure_link_init(sl, &k, now_us);
  memset(&sl->session, 0, sizeof(sl->session));
  memset(peer, 0, sizeof(*peer));

  memset(sl->session.send_key, 0xa5, NOISE_KEYLEN);
  memset(sl->session.recv_key, 0x5a, NOISE_KEYLEN);
  memcpy(peer->recv_key, sl->session.send_key, NOISE_KEYLEN);
  memcpy(peer->send_key, sl->session.recv_key, NOISE_KEYLEN);

  sl->state = SECURE_LINK_ESTABLISHED;
  sl->last_open_us = now_us;
}

static void test_backoff_widens_and_settles(void)
{
  struct secure_link sl;
  struct secure_link_keys k = dummy_keys();
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;
  uint64_t last = 0;
  uint64_t gaps[8];
  int sends = 0;

  secure_link_init(&sl, &k, t);

  /* Twenty seconds of a ground station that never answers. */
  for (uint64_t step = 0; step < 20000000ULL; step += 5000)
    {
      uint64_t now = t + step;
      if (secure_link_poll(&sl, now, out, sizeof(out)) > 0)
        {
          if (sends > 0 && sends <= 8)
            {
              gaps[sends - 1] = now - last;
            }

          last = now;
          sends++;
        }
    }

  CHECK(sends >= 5 && sends <= 9, "expected a handful of retries, got %d", sends);

  /* Backing off means the gaps grow. Jitter adds up to a quarter and is
   * redrawn each time, so consecutive gaps are not strictly ordered: a gap
   * can sit at most 25 percent above its interval, which bounds how far it
   * can fall below the one before it.
   */
  for (int i = 1; i < sends - 1 && i < 8; i++)
    {
      CHECK(gaps[i] * 5 >= gaps[i - 1] * 4,
            "gap %d (%llu) fell too far below gap %d (%llu)",
            i, (unsigned long long)gaps[i], i - 1,
            (unsigned long long)gaps[i - 1]);
    }

  CHECK(gaps[0] < 2 * SECURE_LINK_RETRY_MIN_US, "first retry was not prompt");
  CHECK(gaps[sends - 2] >= SECURE_LINK_RETRY_SLOW_US,
        "never settled at the slow interval");

  CHECK(sl.state == SECURE_LINK_HANDSHAKING, "still handshaking");
}

static void test_silence_triggers_a_rekey(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;

  fake_session(&sl, &peer, t);

  CHECK(secure_link_poll(&sl, t + SECURE_LINK_SILENCE_US - 1, out, sizeof(out)) == 0,
        "rekeyed before the silence timeout");
  CHECK(sl.state == SECURE_LINK_ESTABLISHED, "left established too early");

  CHECK(secure_link_poll(&sl, t + SECURE_LINK_SILENCE_US + 1, out, sizeof(out)) > 0,
        "silence did not produce a handshake");
  CHECK(sl.state == SECURE_LINK_HANDSHAKING, "silence did not rekey");
}

static void test_decrypt_failures_trigger_a_rekey(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t frame[64];
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;

  fake_session(&sl, &peer, t);

  memset(frame, 0, sizeof(frame));
  frame[0] = NOISE_TYPE_TRANSPORT;

  for (uint32_t i = 0; i < SECURE_LINK_DECRYPT_FAILS - 1; i++)
    {
      secure_link_open(&sl, t, frame, sizeof(frame), out, sizeof(out));
      CHECK(sl.state == SECURE_LINK_ESTABLISHED,
            "rekeyed after %u failures, before the threshold", i + 1);
    }

  secure_link_open(&sl, t, frame, sizeof(frame), out, sizeof(out));
  CHECK(sl.state == SECURE_LINK_HANDSHAKING,
        "the threshold did not rekey");
}

static void test_a_replay_is_not_a_decrypt_failure(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t frame[SECURE_LINK_MTU];
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;
  size_t n = 0;

  fake_session(&sl, &peer, t);

  CHECK(noise_session_seal(&peer, (const uint8_t *)"telemetry", 9, frame, &n)
        == NOISE_OK, "peer could not seal");
  CHECK(secure_link_open(&sl, t, frame, n, out, sizeof(out)) == 9,
        "first delivery failed");

  /* A replay is an ordinary radio event, not evidence the peer rekeyed, so
   * however many arrive the session must survive.
   */
  for (uint32_t i = 0; i < SECURE_LINK_DECRYPT_FAILS * 3; i++)
    {
      secure_link_open(&sl, t, frame, n, out, sizeof(out));
    }

  CHECK(sl.state == SECURE_LINK_ESTABLISHED,
        "replays rekeyed the session");
}

static void test_counter_exhaustion_ends_the_session(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;

  fake_session(&sl, &peer, t);
  sl.session.tx = UINT64_MAX;

  CHECK(secure_link_seal(&sl, t, (const uint8_t *)"x", 1, out, sizeof(out))
        == NOISE_ERR_EXHAUSTED, "exhaustion was not reported");
  CHECK(sl.state == SECURE_LINK_HANDSHAKING,
        "exhaustion did not end the session");
}

static void test_nothing_is_sealed_before_a_session(void)
{
  struct secure_link sl;
  struct secure_link_keys k = dummy_keys();
  uint8_t out[SECURE_LINK_MTU];

  secure_link_init(&sl, &k, 1000000);

  CHECK(secure_link_seal(&sl, 1000000, (const uint8_t *)"x", 1, out, sizeof(out)) < 0,
        "sealed without a session");
  CHECK(!secure_link_is_up(&sl), "reported up while handshaking");
}

int main(void)
{
  test_backoff_widens_and_settles();
  test_silence_triggers_a_rekey();
  test_decrypt_failures_trigger_a_rekey();
  test_a_replay_is_not_a_decrypt_failure();
  test_counter_exhaustion_ends_the_session();
  test_nothing_is_sealed_before_a_session();

  printf(failures ? "%d failure(s)\n" : "all state machine tests passed\n",
         failures);
  return failures ? 1 : 0;
}
