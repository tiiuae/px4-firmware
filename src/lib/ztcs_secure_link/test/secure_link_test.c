/****************************************************************************
 * Unit tests for the session state machine. No sockets and no real clock:
 * the library is sans-io precisely so a flight controller's timing can be
 * driven rather than waited for.
 ****************************************************************************/

#include "../secure_link.h"

#include <assert.h>
#include <sodium.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static int failures;
static bool random_fails;

int noise_random(uint8_t *out, size_t len)
{
  if (random_fails)
    {
      return -1;
    }

  randombytes_buf(out, len);
  return 0;
}

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
  memset(k.link.sk, 0x11, sizeof(k.link.sk));
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
  uint8_t a[NOISE_KEYLEN];
  uint8_t b[NOISE_KEYLEN];

  secure_link_init(sl, &k, now_us);
  memset(&sl->session, 0, sizeof(sl->session));
  memset(peer, 0, sizeof(*peer));

  memset(a, 0xa5, sizeof(a));
  memset(b, 0x5a, sizeof(b));
  noise_session_key_set(&sl->session.send, a);
  noise_session_key_set(&sl->session.recv, b);
  noise_session_key_set(&peer->recv, a);
  noise_session_key_set(&peer->send, b);

  sl->state = SECURE_LINK_ESTABLISHED;
  sl->last_open_us = now_us;
  sl->established_us = now_us;
  sl->next_rekey_us = now_us + SECURE_LINK_REKEY_US;
}

/* A backend that holds keys by index has few slots; a leak shows as the next
 * test failing to key.
 */
static void end_session(struct secure_link *sl, struct noise_session *peer)
{
  secure_link_close(sl);
  noise_session_wipe(peer);
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

  end_session(&sl, &peer);
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

  end_session(&sl, &peer);
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

  end_session(&sl, &peer);
}

static void test_a_frame_opens_into_a_buffer_the_size_of_its_plaintext(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t msg[600];
  uint8_t frame[NOISE_TRANSPORT_HDR_LEN + sizeof(msg) + NOISE_TAGLEN];
  uint8_t out[sizeof(msg)];
  uint64_t t = 1000000;
  size_t n = 0;

  fake_session(&sl, &peer, t);
  memset(msg, 0x42, sizeof(msg));

  CHECK(noise_session_seal(&peer, msg, sizeof(msg), frame, &n) == NOISE_OK,
        "peer could not seal");
  CHECK(secure_link_open(&sl, t, frame, n, out, sizeof(out)) == (int)sizeof(msg),
        "a full-size reply was rejected");
  CHECK(memcmp(out, msg, sizeof(msg)) == 0, "plaintext mismatch");
  CHECK(secure_link_open(&sl, t, frame, n, out, sizeof(out) - 1) < 0,
        "an undersized buffer was accepted");

  end_session(&sl, &peer);
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

  end_session(&sl, &peer);
}

static void test_a_rekey_runs_while_the_session_carries_on(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;
  uint64_t due = t + SECURE_LINK_REKEY_US;

  fake_session(&sl, &peer, t);
  sl.last_open_us = due - 1;

  CHECK(secure_link_poll(&sl, due - 1, out, sizeof(out)) == 0,
        "rekeyed early");

  CHECK(secure_link_poll(&sl, due, out, sizeof(out)) == NOISE_MSG1_LEN,
        "no rekey handshake at the interval");
  CHECK(sl.state == SECURE_LINK_ESTABLISHED, "the rekey took the link down");
  CHECK(secure_link_seal(&sl, due, (const uint8_t *)"x", 1, out, sizeof(out)) > 0,
        "nothing sealed under the old session during the rekey");
  CHECK(secure_link_poll(&sl, due + 1, out, sizeof(out)) == 0,
        "the rekey handshake was resent at once");
  end_session(&sl, &peer);
}

static void test_a_session_never_renewed_ends_at_its_max_age(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t out[SECURE_LINK_MTU];
  uint64_t t = 1000000;
  uint64_t old = t + SECURE_LINK_MAX_AGE_US;

  fake_session(&sl, &peer, t);
  sl.last_open_us = old;

  CHECK(secure_link_poll(&sl, old, out, sizeof(out)) == NOISE_MSG1_LEN,
        "no fresh handshake at the max age");
  CHECK(sl.state == SECURE_LINK_HANDSHAKING, "an unrenewed session outlived its max age");
  CHECK(secure_link_seal(&sl, old, (const uint8_t *)"x", 1, out, sizeof(out)) < 0,
        "sealed under a session past its max age");
  end_session(&sl, &peer);
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

static void test_no_entropy_after_silence_seals_nothing(void)
{
  struct secure_link sl;
  struct noise_session peer;
  uint8_t out[SECURE_LINK_MTU];
  uint64_t now = 1000000 + SECURE_LINK_SILENCE_US + 1;

  fake_session(&sl, &peer, 1000000);
  random_fails = true;

  CHECK(secure_link_poll(&sl, now, out, sizeof(out)) == NOISE_ERR_RANDOM,
        "a handshake left without entropy");
  CHECK(!secure_link_is_up(&sl), "up on a wiped session");
  CHECK(secure_link_seal(&sl, now, (const uint8_t *)"x", 1, out, sizeof(out)) < 0,
        "sealed under a wiped session");

  random_fails = false;
  end_session(&sl, &peer);
}

static void test_no_entropy_retries_on_the_backoff(void)
{
  struct secure_link sl;
  struct secure_link_keys k = dummy_keys();
  uint8_t out[SECURE_LINK_MTU];
  uint64_t now = 1000000;
  int attempts = 0;
  int n = 0;

  secure_link_init(&sl, &k, now);
  random_fails = true;

  for (; now < 21000000; now += 5000)
    {
      attempts += secure_link_poll(&sl, now, out, sizeof(out)) == NOISE_ERR_RANDOM;
    }

  CHECK(attempts >= 5 && attempts <= 9, "expected a handful of attempts, got %d", attempts);

  random_fails = false;

  for (; now < 81000000 && n <= 0; now += 5000)
    {
      n = secure_link_poll(&sl, now, out, sizeof(out));
    }

  CHECK(n == NOISE_MSG1_LEN, "no handshake once entropy returned");
  secure_link_close(&sl);
}

int main(void)
{
  test_backoff_widens_and_settles();
  test_silence_triggers_a_rekey();
  test_decrypt_failures_trigger_a_rekey();
  test_a_replay_is_not_a_decrypt_failure();
  test_a_frame_opens_into_a_buffer_the_size_of_its_plaintext();
  test_counter_exhaustion_ends_the_session();
  test_a_rekey_runs_while_the_session_carries_on();
  test_a_session_never_renewed_ends_at_its_max_age();
  test_nothing_is_sealed_before_a_session();
  test_no_entropy_after_silence_seals_nothing();
  test_no_entropy_retries_on_the_backoff();

  printf(failures ? "%d failure(s)\n" : "all state machine tests passed\n",
         failures);
  return failures ? 1 : 0;
}
