#ifndef ZTCS_CHACHA20_IETF_H
#define ZTCS_CHACHA20_IETF_H

#include <stddef.h>
#include <stdint.h>

void chacha20_ietf_block(const uint8_t key[32], const uint8_t nonce[12],
                         uint32_t counter, uint8_t out[64]);

void chacha20_ietf_xor(const uint8_t key[32], const uint8_t nonce[12],
                       uint32_t counter, uint8_t *data, size_t len);

#endif
