/* ML-KEM-768 behind noise_backend.h, on mlkem-native. The library is
 * portable C, so the host and the aircraft run the same code; what the
 * aircraft adds is a thread to run it on, because the KEM needs more stack
 * than a syscall has. Everything else in the handshake stays on the caller's
 * thread, where the session key slots are owned.
 *
 * This file is the ZTCS one plus that thread. See mlkem-native/VENDOR.md for
 * the pin, the build flags and why this implementation.
 */

#include "noise_backend.h"

#ifdef NOISE_HFS

#include <string.h>

#include "mlkem_native.h"

/* A compile that disagrees with the wire format is better than a handshake
 * that disagrees with the station.
 */
#if MLKEM768_PUBLICKEYBYTES != NOISE_KEM_PUBLEN || \
    MLKEM768_CIPHERTEXTBYTES != NOISE_KEM_CTLEN || \
    MLKEM768_BYTES != NOISE_KEM_SSLEN
#error "mlkem-native is not configured for ML-KEM-768"
#endif

static int kem_public(const uint8_t seed[NOISE_KEM_SEEDLEN],
                      uint8_t pk[NOISE_KEM_PUBLEN]) {
  uint8_t sk[MLKEM768_SECRETKEYBYTES];
  int rc = ztcs_mlkem_keypair_derand(pk, sk, seed);
  noise_wipe(sk, sizeof(sk));
  return rc == 0 ? 0 : -1;
}

static int kem_decap(const uint8_t seed[NOISE_KEM_SEEDLEN],
                     const uint8_t ct[NOISE_KEM_CTLEN],
                     uint8_t ss[NOISE_KEM_SSLEN]) {
  uint8_t pk[MLKEM768_PUBLICKEYBYTES];
  uint8_t sk[MLKEM768_SECRETKEYBYTES];
  int rc = ztcs_mlkem_keypair_derand(pk, sk, seed);
  if (rc == 0) {
    rc = ztcs_mlkem_dec(ss, ct, sk);
  }
  noise_wipe(sk, sizeof(sk));
  return rc == 0 ? 0 : -1;
}

int noise_kem_encap(const uint8_t pk[NOISE_KEM_PUBLEN],
                    const uint8_t coins[NOISE_KEM_COINLEN],
                    uint8_t ct[NOISE_KEM_CTLEN], uint8_t ss[NOISE_KEM_SSLEN]) {
  return ztcs_mlkem_enc_derand(ct, ss, pk, coins) == 0 ? 0 : -1;
}

#if defined(PX4_NOISE_KERNEL)

/* Measured: 18.6 KB to generate a keypair and 22.9 KB to decapsulate,
 * against the 8 KB a syscall runs on. One thread with a stack of its own
 * serves every handshake, which costs one stack rather than one per task.
 * The caller blocks, so nothing else moves off its own thread: the session
 * key slots are owned per process and must stay that way.
 */

#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <sched.h>
#include <semaphore.h>

#ifndef CONFIG_PX4_NOISE_KEM_STACKSIZE
#define CONFIG_PX4_NOISE_KEM_STACKSIZE 32768
#endif

enum kem_op { KEM_PUBLIC, KEM_DECAP };

static struct {
  sem_t request;
  sem_t done;
  enum kem_op op;
  const uint8_t *seed;
  const uint8_t *ct;
  uint8_t *out;
  int rc;
} g_job;

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static bool g_ready;

static int kem_thread(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  for (;;)
    {
      while (sem_wait(&g_job.request) != 0)
        {
        }

      g_job.rc = g_job.op == KEM_PUBLIC
                 ? kem_public(g_job.seed, g_job.out)
                 : kem_decap(g_job.seed, g_job.ct, g_job.out);
      sem_post(&g_job.done);
    }

  return 0;
}

static int kem_run(enum kem_op op, const uint8_t *seed, const uint8_t *ct,
                   uint8_t *out)
{
  int rc;

  nxmutex_lock(&g_lock);

  if (!g_ready)
    {
      sem_init(&g_job.request, 0, 0);
      sem_init(&g_job.done, 0, 0);

      if (kthread_create("noise_kem", SCHED_PRIORITY_DEFAULT,
                         CONFIG_PX4_NOISE_KEM_STACKSIZE, kem_thread, NULL) < 0)
        {
          nxmutex_unlock(&g_lock);
          return -1;
        }

      g_ready = true;
    }

  g_job.op = op;
  g_job.seed = seed;
  g_job.ct = ct;
  g_job.out = out;
  sem_post(&g_job.request);

  while (sem_wait(&g_job.done) != 0)
    {
    }

  rc = g_job.rc;
  nxmutex_unlock(&g_lock);
  return rc;
}

int noise_kem_public(const uint8_t seed[NOISE_KEM_SEEDLEN],
                     uint8_t pk[NOISE_KEM_PUBLEN]) {
  return kem_run(KEM_PUBLIC, seed, NULL, pk);
}

int noise_kem_decap(const uint8_t seed[NOISE_KEM_SEEDLEN],
                    const uint8_t ct[NOISE_KEM_CTLEN],
                    uint8_t ss[NOISE_KEM_SSLEN]) {
  return kem_run(KEM_DECAP, seed, ct, ss);
}

#else

int noise_kem_public(const uint8_t seed[NOISE_KEM_SEEDLEN],
                     uint8_t pk[NOISE_KEM_PUBLEN]) {
  return kem_public(seed, pk);
}

int noise_kem_decap(const uint8_t seed[NOISE_KEM_SEEDLEN],
                    const uint8_t ct[NOISE_KEM_CTLEN],
                    uint8_t ss[NOISE_KEM_SSLEN]) {
  return kem_decap(seed, ct, ss);
}

#endif

#endif
