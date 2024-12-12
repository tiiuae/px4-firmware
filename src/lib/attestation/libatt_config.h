/****************************************************************************
 * Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
 ****************************************************************************/

#ifndef __LIBATT_DEFAULT_CONFIG_H
#define __LIBATT_DEFAULT_CONFIG_H

/*
 *
 * libatt customization
 *
 * Set the `LIBATT_CONFIG_H` compiler definition on libatt to use your own
 * version of this file.
 */


/*
 * Some platform-specific types and functions to use
 */
#include <nuttx/mutex.h>
#define LIBATT_MUTEX_T mutex_t
#define LIBATT_MUTEX_INITIALIZER NXMUTEX_INITIALIZER
#define LIBATT_MUTEX_LOCK_F nxmutex_lock
#define LIBATT_MUTEX_UNLOCK_F nxmutex_unlock

#include <stdlib.h>
#define LIBATT_MALLOC_F malloc
#define LIBATT_CALLOC_F calloc
#define LIBATT_REALLOC_F realloc
#define LIBATT_FREE_F free

#include <stdio.h>
#define LIBATT_PRINTF_F printf
#define LIBATT_FPRINTF_F fprintf
#define LIBATT_FFLUSH_F fflush


/**
 * \def LIBATT_PRV_VERIFY_DAC
 *
 * If the DAC is retrieved from an untrusted source (e.g. passed by the
 * REE somehow or read from unprotected storage), define this to verify its
 * signature before using it to verify a connecting peer.
 *
 * This might prove quite a substiantial overhead (several mallocs, json
 * parsing, and base64 decoding especially when LIBATT_DAC_VERIFY_THEN_PARSE is not
 * set) during (at least) the first session establishment, so better to provide
 * the prover with a pre-verified, trusted DAC, if possible. If we want to avoid
 * the parsing altogether, we need to implement providing the DAC in another
 * form.
 */
#define LIBATT_PRV_VERIFY_DAC

/**
 * \def LIBATT_DAC_VERIFY_THEN_PARSE
 *
 * When defined, the DAC is given to the prover in compact form (no
 * whitespace) ready to be validated. This removes the attack surface of the
 * JSON parsing and compacting logic on an untrusted DAC input. Once the
 * signature is verified, only then the JSON logic gets applied. At that point,
 * the risk is far lower (if any, assuming DAC signing keys are not leaked).
 *
 * Having this setting undefined, allows the input DAC to be pretty printed, and
 * in its full form (containing also the 'auth' section). The prover will find
 * the device_assembly_card object, compact it (remove whitespaces), parse the
 * signing certificate + the signature, and then verify it.
 *
 * If `LIBATT_PRV_VERIFY_DAC` is undefined, this has no effect.
 */
#define LIBATT_DAC_VERIFY_THEN_PARSE

/**
 * \def LIBATT_FORMAT_PEM
 *
 * Certificates and keys are provided in PEM format. This results in
 * MbedTLS parsing certificates using dynamic allocations. Undefining this
 * option causes the parsing to assume input format to be DER, and enables the
 * use of `mbedtls_x509_crt_parse_der_nocopy()` for certificates (keys are still
 * parsed using dynamic allocations).
 *
 * See also:
 * https://mbed-tls.readthedocs.io/en/latest/kb/how-to/reduce-polarssl-memory-and-storage-footprint/#parsing-x-509-certificates-without-copying-the-raw-certificate-data
 *
 * \note Having this defined, yet providing keys or certificates in DER format
 * is supported. The opposite is not.
 */
// #define LIBATT_FORMAT_PEM

/**
 * \def LIBATT_TLV_PRINTING_ENABLED
 *
 * Enables the TLV printing functions, tag names, and as a result enables also
 * the `list` and `query` commands on the FMO/challenger console.
 */
#define LIBATT_TLV_PRINTING_ENABLED

#endif /* __LIBATT_DEFAULT_CONFIG_H */
