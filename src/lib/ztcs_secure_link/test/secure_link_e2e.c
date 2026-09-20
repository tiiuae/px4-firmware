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

#include "../secure_link.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define POLL_INTERVAL_US 20000ULL
#define DEADLINE_US      30000000ULL

static uint64_t now_us(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
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
  uint8_t buf[SECURE_LINK_MTU];
  uint8_t plain[SECURE_LINK_MTU];
  uint64_t start;
  uint64_t quiet_until = 0;
  unsigned handshakes_at_send = 0;
  int round = 0;
  int rounds;
  bool sent = false;
  int fd;

  if (argc < 7 || argc > 8
      || unhex(argv[3], keys.link.sk, NOISE_DHLEN)
      || unhex(argv[4], keys.station_public, NOISE_DHLEN)
      || unhex(argv[5], keys.identity, NOISE_IDENTITY_PAYLOAD_LEN))
    {
      fprintf(stderr, "usage: secure_link_e2e <host> <port> <s_priv> "
                      "<station_pub> <identity> <msg> [rounds]\n");
      return 2;
    }

  rounds = argc == 8 ? atoi(argv[7]) : 1;

  memset(&to, 0, sizeof(to));
  to.sin_family = AF_INET;
  to.sin_port = htons((uint16_t)atoi(argv[2]));
  if (inet_pton(AF_INET, argv[1], &to.sin_addr) != 1)
    {
      fprintf(stderr, "bad host %s\n", argv[1]);
      return 2;
    }

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0 || connect(fd, (struct sockaddr *)&to, sizeof(to)) != 0)
    {
      perror("socket");
      return 1;
    }

  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
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
          send(fd, buf, (size_t)n, 0);
        }

      if (secure_link_is_up(&sl) && !sent && t >= quiet_until)
        {
          char payload[64];
          snprintf(payload, sizeof(payload), "%s-%d", argv[6], round + 1);
          n = secure_link_seal(&sl, t, (const uint8_t *)payload,
                               strlen(payload), buf, sizeof(buf));
          if (n > 0)
            {
              send(fd, buf, (size_t)n, 0);
              printf("uplink %s\n", payload);
              handshakes_at_send = sl.handshakes;
              sent = true;
            }
        }

      got = recv(fd, buf, sizeof(buf), 0);
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
              close(fd);
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
  close(fd);
  return 1;
}
