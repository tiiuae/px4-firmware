/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

__BEGIN_DECLS

/* The distinguished name a CSR carries, laid out as the enrolment agent
 * passes it. Fixed arrays, NUL padded, because this crosses an FFI boundary
 * from no_std Rust.
 */
struct ele_csr_req {
	char country[16];
	char locality[64];
	char email[64];
	char org[64];
	char state[64];
	char orgunit[64];
};

/**
 * Build a PKCS#10 certificate request for the identity key held in the
 * enclave, signed by it.
 *
 * The private half never leaves the enclave: this side builds the request
 * body, hands the enclave its digest, and wraps the signature that comes
 * back. The board is asking to be certified for a key it cannot itself read.
 *
 * @param req    the distinguished name
 * @param buf    receives a NUL terminated PEM document
 * @param buflen its size
 * @return zero, or a negative errno.
 */
int ele_csr_create(const struct ele_csr_req *req, char *buf, size_t buflen);

__END_DECLS
