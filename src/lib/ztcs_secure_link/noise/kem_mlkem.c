/* ML-KEM-768 behind noise_backend.h, on mlkem-native. The same file on the
 * host and on the aircraft: the library is portable C and the only thing
 * that changes between them is which backend compiles beside it.
 *
 * See mlkem-native/VENDOR.md for the pin, the build flags and why this
 * implementation.
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

int noise_kem_public(const uint8_t seed[NOISE_KEM_SEEDLEN],
                     uint8_t pk[NOISE_KEM_PUBLEN]) {
  uint8_t sk[MLKEM768_SECRETKEYBYTES];
  int rc = ztcs_mlkem_keypair_derand(pk, sk, seed);
  noise_wipe(sk, sizeof(sk));
  return rc == 0 ? 0 : -1;
}

int noise_kem_decap(const uint8_t seed[NOISE_KEM_SEEDLEN],
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

#endif
