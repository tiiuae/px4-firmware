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
#include <pthread.h>
#define LIBATT_MUTEX_T           pthread_mutex_t
#define LIBATT_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER
#define LIBATT_MUTEX_LOCK_F      pthread_mutex_lock
#define LIBATT_MUTEX_UNLOCK_F    pthread_mutex_unlock

#include <stdlib.h>
#define LIBATT_MALLOC_F  malloc
#define LIBATT_CALLOC_F  calloc
#define LIBATT_FREE_F    free

/* optional; undefine to use naive internal implemenation */
#define LIBATT_REALLOC_F realloc

#include <stdio.h>
#define LIBATT_SNPRINTF_F   snprintf
#define LIBATT_INFO_F       printf
#define LIBATT_DEBUG_F      printf
#define LIBATT_ERROR_F(...) fprintf(stderr, __VA_ARGS__)

/**
 * \def LIBATT_NO_STRERROR
 *
 * Define in environments without `strerror()` or <errno.h>
 */
// #define LIBATT_NO_STRERROR

/**
 * \def LIBATT_ERROR_STRINGS_ENABLED
 *
 * When defined `libatt_strerr(code)` returns descriptive error messages
 * instead of just hex codes. By default enabled on debug builds (NDEBUG
 * undefined), in which case if you want to disable it, you need to undefine
 * it.
 */
#define LIBATT_ERROR_STRINGS_ENABLED

/**
 * \def LIBATT_DAC_VERIFY_THEN_PARSE
 *
 * When defined, the DAC is given to the prover in compact form (no
 * whitespace) ready to be validated. This removes the attack surface of the
 * JSON parsing and compacting logic on an untrusted DAC input. Once the
 * signature is verified, only then the JSON logic gets applied. At that
 * point, the risk is far lower (if any, assuming DAC signing keys are not
 * leaked).
 *
 * Having this setting undefined, allows the input DAC to be pretty printed,
 * and in its full form (containing also the 'auth' section). The prover
 * will find the device_assembly_card object, compact it (remove
 * whitespaces), parse the signing certificate + the signature, and then
 * verify it.
 */
#define LIBATT_DAC_VERIFY_THEN_PARSE

/**
 * \def LIBATT_TLV_PRINTING_ENABLED
 *
 * Enables the TLV printing functions, tag names, and as a result enables
 * also the `list` and `query` commands on the FMO/challenger console. By
 * default enabled on debug builds (NDEBUG undefined), in which case if you
 * want to disable it, you need to undefine it.
 */
#define LIBATT_TLV_PRINTING_ENABLED

#endif /* __LIBATT_DEFAULT_CONFIG_H */
