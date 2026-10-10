/****************************************************************************
 * Drives the flight-controller state machine against a real ground station
 * over a real socket. Host only: the point is to exercise secure_link.c
 * itself, not a reimplementation of it, so what ships is what was tested.
 *
 *   secure_link_e2e <host> <port> <s_priv> <station_pub> <identity> <msg>
 *                   [rounds]
 *
 * With `rounds` above one it goes quiet between exchanges for longer than
 * the silence timeout, which is how a radio dropout looks from here: the
 * link must rekey itself and the ground station must follow without the old
 * session getting in the way.
 ****************************************************************************/

#include "../cobs.h"
#include "../secure_link.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#define POLL_INTERVAL_US 20000ULL
#define DEADLINE_US      30000000ULL

static uint64_t now_us(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static int g_fd;
static bool g_serial;
static struct cobs_rx g_rx;

static void tx(const uint8_t *frame, size_t len)
{
  uint8_t out[COBS_ENCODED_MAX(SECURE_LINK_FRAME_MAX) + 2];
  size_t n;

  if (!g_serial)
    {
      send(g_fd, frame, len, 0);
      return;
    }

  n = cobs_encode(frame, len, out + 1);
  out[0] = 0;
  out[n + 1] = 0;
  if (write(g_fd, out, n + 2) != (ssize_t)(n + 2))
    {
      perror("write");
    }
}

static ssize_t rx(uint8_t *frame, size_t cap)
{
  struct pollfd p = {g_fd, POLLIN, 0};
  uint8_t byte;

  if (!g_serial)
    {
      return recv(g_fd, frame, cap, 0);
    }

  while (poll(&p, 1, POLL_INTERVAL_US / 1000) > 0)
    {
      int n;

      if (read(g_fd, &byte, 1) != 1)
        {
          return -1;
        }

      n = cobs_rx_push(&g_rx, byte, frame, cap);

      if (n > 0)
        {
          return n;
        }
    }

  return 0;
}

static int open_serial(const char *path)
{
  struct termios t;

  g_fd = open(path, O_RDWR | O_NOCTTY);

  if (g_fd < 0 || tcgetattr(g_fd, &t) != 0)
    {
      return -1;
    }

  cfmakeraw(&t);
  return tcsetattr(g_fd, TCSANOW, &t);
}

static int unhex(const char *s, uint8_t *out, size_t want)
{
  if (strlen(s) != want * 2)
    {
      return -1;
    }

  for (size_t i = 0; i < want; i++)
    {
      unsigned v;
      if (sscanf(s + 2 * i, "%2x", &v) != 1)
        {
          return -1;
        }

      out[i] = (uint8_t)v;
    }

  return 0;
}

int main(int argc, char **argv)
{
  struct secure_link sl;
  struct secure_link_keys keys;
  struct sockaddr_in to;
  struct timeval tv = {0, POLL_INTERVAL_US};
  uint8_t buf[SECURE_LINK_FRAME_MAX];
  uint8_t plain[SECURE_LINK_FRAME_MAX];
  uint64_t start;
  uint64_t quiet_until = 0;
  unsigned handshakes_at_send = 0;
  int round = 0;
  int rounds;
  bool sent = false;
  if (argc < 7 || argc > 8
      || unhex(argv[3], keys.link.sk, NOISE_DHLEN)
      || unhex(argv[4], keys.station_public, NOISE_DHLEN)
      || unhex(argv[5], keys.identity, NOISE_IDENTITY_PAYLOAD_LEN))
    {
      fprintf(stderr, "usage: secure_link_e2e <host|/dev/tty> <port> <s_priv> "
                      "<station_pub> <identity> <msg> [rounds]\n");
      return 2;
    }

  rounds = argc == 8 ? atoi(argv[7]) : 1;

  g_serial = argv[1][0] == '/';

  if (g_serial)
    {
      if (open_serial(argv[1]) != 0)
        {
          perror(argv[1]);
          return 1;
        }
    }
  else
    {
      memset(&to, 0, sizeof(to));
      to.sin_family = AF_INET;
      to.sin_port = htons((uint16_t)atoi(argv[2]));
      if (inet_pton(AF_INET, argv[1], &to.sin_addr) != 1)
        {
          fprintf(stderr, "bad host %s\n", argv[1]);
          return 2;
        }

      g_fd = socket(AF_INET, SOCK_DGRAM, 0);
      if (g_fd < 0
          || connect(g_fd, (struct sockaddr *)&to, sizeof(to)) != 0)
        {
          perror("socket");
          return 1;
        }

      setsockopt(g_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
  secure_link_init(&sl, &keys, now_us());
  start = now_us();

  while (now_us() - start < DEADLINE_US)
    {
      uint64_t t = now_us();
      ssize_t got;
      int n;

      /* Retransmit and rekey both come out of here, which is why it runs
       * whether or not there is traffic.
       */
      n = secure_link_poll(&sl, t, buf, sizeof(buf));
      if (n > 0)
        {
          printf("handshake %u\n", sl.handshakes);
          tx(buf, (size_t)n);
        }

      if (secure_link_is_up(&sl) && !sent && t >= quiet_until)
        {
          char payload[64];
          uint8_t mav[6 + sizeof(payload) + 2] = {0xfe, 0, 0, 1, 1, 0};
          snprintf(payload, sizeof(payload), "%s-%d", argv[6], round + 1);
          mav[1] = (uint8_t)strlen(payload);
          memcpy(mav + 6, payload, mav[1]);
          n = secure_link_seal(&sl, t, mav, 6u + mav[1] + 2u,
                               buf, sizeof(buf));
          if (n > 0)
            {
              tx(buf, (size_t)n);
              printf("uplink %s\n", payload);
              handshakes_at_send = sl.handshakes;
              sent = true;
            }
        }

      got = rx(buf, sizeof(buf));
      if (got <= 0)
        {
          continue;
        }

      n = secure_link_open(&sl, now_us(), buf, (size_t)got, plain,
                           sizeof(plain));
      if (n > 0)
        {
          printf("downlink %.*s\n", n, (const char *)plain);

          if (++round >= rounds)
            {
              close(g_fd);
              return 0;
            }

          /* Go quiet for longer than the silence timeout. Nothing opens in
           * that window, which is exactly what a dropout looks like.
           */
          sent = false;
          quiet_until = now_us() + SECURE_LINK_SILENCE_US * 2;
          printf("quiet for %llu us\n",
                 (unsigned long long)(SECURE_LINK_SILENCE_US * 2));
        }

      if (n == 0 && secure_link_is_up(&sl))
        {
          printf("keyed after %u handshake(s)\n", sl.handshakes);

          if (round > 0 && sl.handshakes > handshakes_at_send)
            {
              printf("rekeyed after the dropout\n");
            }
        }
    }

  fprintf(stderr, "deadline reached in state %d\n", (int)sl.state);
  close(g_fd);
  return 1;
}
