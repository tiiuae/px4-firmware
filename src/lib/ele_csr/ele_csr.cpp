/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file ele_csr.cpp
 *
 * A PKCS#10 certificate request for a key the enclave holds. This side builds
 * the request body, the enclave signs its digest, and the signature is
 * wrapped here. The private half is never read, because it cannot be.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <px4_platform_common/crypto.h>

#include "ele_csr.h"

/* The enclave hashes the body itself: user space has no SHA-256. */
#define ELE_IDENTITY_MSG_INDEX 0xe1
#define PUB_LEN                64
#define SIG_LEN                64

namespace
{

/* A DER writer that fills from the end, because a SEQUENCE has to know the
 * length of its contents before it can write its own header.
 */
class Der
{
public:
	Der(uint8_t *buf, size_t len) : _buf(buf), _end(buf + len), _p(buf + len) {}

	size_t len() const { return (size_t)(_end - _p); }
	const uint8_t *data() const { return _p; }
	bool bad() const { return _bad; }

	void raw(const uint8_t *b, size_t n)
	{
		if ((size_t)(_p - _buf) < n) { _bad = true; return; }

		_p -= n;
		memcpy(_p, b, n);
	}

	void byte(uint8_t b) { raw(&b, 1); }

	void length(size_t n)
	{
		if (n < 0x80) { byte((uint8_t)n); return; }

		/* DER lengths are big endian, and this writes back to front, so
		 * the least significant byte goes down first.
		 */
		uint8_t n_bytes = 0;

		for (size_t v = n; v; v >>= 8) { n_bytes++; }

		for (uint8_t i = 0; i < n_bytes; i++) {
			byte((uint8_t)((n >> (8 * i)) & 0xff));
		}

		byte((uint8_t)(0x80 | n_bytes));
	}

	/* Wrap everything written since mark into tag. */
	void wrap(uint8_t tag, size_t mark)
	{
		length(len() - mark);
		byte(tag);
	}

	void oid(const uint8_t *o, size_t n)
	{
		raw(o, n);
		length(n);
		byte(0x06);
	}

	void utf8(const char *s)
	{
		size_t n = strlen(s);
		raw((const uint8_t *)s, n);
		length(n);
		byte(0x0c);
	}

private:
	uint8_t *_buf;
	uint8_t *_end;
	uint8_t *_p;
	bool _bad {false};
};

const uint8_t oid_c[]     = {0x55, 0x04, 0x06};
const uint8_t oid_st[]    = {0x55, 0x04, 0x08};
const uint8_t oid_l[]     = {0x55, 0x04, 0x07};
const uint8_t oid_o[]     = {0x55, 0x04, 0x0a};
const uint8_t oid_ou[]    = {0x55, 0x04, 0x0b};
const uint8_t oid_email[] = {0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x09, 0x01};

/* AlgorithmIdentifier for ecdsa-with-SHA256, no parameters. */
const uint8_t alg_ecdsa_sha256[] = {
	0x30, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02
};

/* SubjectPublicKeyInfo prefix for id-ecPublicKey over prime256v1, up to and
 * including the BIT STRING header and the uncompressed point marker.
 */
const uint8_t spki_p256[] = {
	0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02,
	0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
	0x42, 0x00, 0x04
};

void rdn(Der &d, const uint8_t *o, size_t olen, const char *value)
{
	if (value[0] == '\0') {
		return;
	}

	size_t mark = d.len();
	d.utf8(value);
	d.oid(o, olen);
	d.wrap(0x30, mark);
	d.wrap(0x31, mark);
}

/* r and s as DER INTEGERs, from the raw pair the enclave returns. */
void ecdsa_integer(Der &d, const uint8_t *v)
{
	size_t n = 32;

	while (n > 1 && v[32 - n] == 0) { n--; }

	bool pad = (v[32 - n] & 0x80) != 0;
	d.raw(v + 32 - n, n);

	if (pad) { d.byte(0x00); }

	d.length(n + (pad ? 1 : 0));
	d.byte(0x02);
}

const char b64set[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t b64encode(const uint8_t *in, size_t inlen, char *out)
{
	char *p = out;

	for (size_t i = 0; i < inlen; i += 3) {
		uint32_t v = (uint32_t)in[i] << 16;
		size_t n = inlen - i;

		v |= (n > 1 ? (uint32_t)in[i + 1] << 8 : 0);
		v |= (n > 2 ? (uint32_t)in[i + 2] : 0);

		*p++ = b64set[(v >> 18) & 0x3f];
		*p++ = b64set[(v >> 12) & 0x3f];
		*p++ = n > 1 ? b64set[(v >> 6) & 0x3f] : '=';
		*p++ = n > 2 ? b64set[v & 0x3f] : '=';
	}

	return (size_t)(p - out);
}

int pem(const uint8_t *der, size_t derlen, char *out, size_t outlen)
{
	static const char *head = "-----BEGIN CERTIFICATE REQUEST-----\n";
	static const char *tail = "-----END CERTIFICATE REQUEST-----\n";
	static char b64[1536];

	if ((derlen + 2) / 3 * 4 > sizeof(b64)) {
		return -E2BIG;
	}

	size_t b64len = b64encode(der, derlen, b64);
	size_t need = strlen(head) + b64len + (b64len / 64 + 2) + strlen(tail) + 1;

	if (need > outlen) {
		return -E2BIG;
	}

	char *p = out;
	strcpy(p, head);
	p += strlen(head);

	for (size_t i = 0; i < b64len; i += 64) {
		size_t n = (b64len - i < 64) ? b64len - i : 64;
		memcpy(p, b64 + i, n);
		p += n;
		*p++ = '\n';
	}

	strcpy(p, tail);
	return 0;
}

} // namespace

int ele_csr_create(const struct ele_csr_req *req, char *buf, size_t buflen)
{
	PX4Crypto crypto;
	uint8_t pub[PUB_LEN];
	uint8_t sig[SIG_LEN];
	static uint8_t der[1024];
	static uint8_t info[1024];
	size_t publen = sizeof(pub);

	if (req == nullptr || buf == nullptr) {
		return -EINVAL;
	}

	if (!crypto.open(CRYPTO_ECDSA_P256)) {
		return -ENODEV;
	}

	if (!crypto.get_public_key(ELE_IDENTITY_MSG_INDEX, pub, &publen) || publen != PUB_LEN) {
		return -ENOENT;
	}

	/* CertificationRequestInfo, built back to front. */
	Der d(der, sizeof(der));
	size_t info_mark = d.len();

	d.byte(0x00);
	d.byte(0xa0);            /* attributes [0], empty */

	d.raw(pub, PUB_LEN);
	d.raw(spki_p256, sizeof(spki_p256));

	size_t name_mark = d.len();
	rdn(d, oid_email, sizeof(oid_email), req->email);
	rdn(d, oid_ou, sizeof(oid_ou), req->orgunit);
	rdn(d, oid_o, sizeof(oid_o), req->org);
	rdn(d, oid_l, sizeof(oid_l), req->locality);
	rdn(d, oid_st, sizeof(oid_st), req->state);
	rdn(d, oid_c, sizeof(oid_c), req->country);
	d.wrap(0x30, name_mark);

	d.byte(0x00);
	d.byte(0x01);
	d.byte(0x02);            /* version 0 */
	d.wrap(0x30, info_mark);

	if (d.bad()) {
		return -E2BIG;
	}

	size_t infolen = d.len();
	memcpy(info, d.data(), infolen);

	/* The enclave hashes what it signs, so it takes the body itself. */
	if (!crypto.sign(ELE_IDENTITY_MSG_INDEX, sig, info, infolen)) {
		return -EIO;
	}

	/* CertificationRequest wraps the body, the algorithm, and the signature. */
	Der o(der, sizeof(der));
	size_t req_mark = o.len();

	size_t sig_mark = o.len();
	ecdsa_integer(o, sig + 32);
	ecdsa_integer(o, sig);
	o.wrap(0x30, sig_mark);
	o.byte(0x00);            /* unused bits */
	o.length(o.len() - sig_mark);
	o.byte(0x03);

	o.raw(alg_ecdsa_sha256, sizeof(alg_ecdsa_sha256));
	o.raw(info, infolen);
	o.wrap(0x30, req_mark);

	if (o.bad()) {
		return -E2BIG;
	}

	return pem(o.data(), o.len(), buf, buflen);
}
