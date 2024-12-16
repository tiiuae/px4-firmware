/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdio.h>

#include <pfsoc/prover_ecall.h>
#include <tee.h>

#include <nuttx/mm/mm.h>
#include <att_malloc.h>

#include <prover/prover.h>

/*
 * TEE-based heap implementation
 */

/* We don't support tee_extend() yet, so provide a fixed size mem pool that
 * <<should>> be enough for most known applications */
#define ATT_HEAP_SIZE (32 * 1024)

static void *g_tee_mempool = NULL;
static struct mm_heap_s *g_tee_heap = NULL;

void *att_malloc(size_t size)
{
	if (!g_tee_mempool || !g_tee_heap) {
		return NULL;
	}

	return mm_malloc(g_tee_heap, size);
}

void *att_realloc(void *ptr, size_t new_size)
{
	if (!g_tee_mempool || !g_tee_heap) {
		return NULL;
	}

	return mm_realloc(g_tee_heap, ptr, new_size);
}

void *att_calloc(size_t nelem, size_t elsize)
{
	if (!g_tee_mempool || !g_tee_heap) {
		return NULL;
	}

	return mm_calloc(g_tee_heap, nelem, elsize);
}

void att_free(void *ptr)
{
	if (!g_tee_mempool || !g_tee_heap) {
		return;
	}

	mm_free(g_tee_heap, ptr);
}

/*
 * TEE-based prover implementation
 */

// *INDENT-OFF*
#define NEW_CALL(fid) { \
		.funcid = (fid), \
		.out = 0, \
		.a[0] = {.v = 0, .size = 0}, \
		.a[1] = {.v = 0, .size = 0}, \
		.a[2] = {.v = 0, .size = 0}, \
		.a[3] = {.v = 0, .size = 0}, \
		.a[4] = {.v = 0, .size = 0}, \
		.a[5] = {.v = 0, .size = 0} \
	}
// *INDENT-ON*

#define INVOKE(args) \
	int ret = tee_call(&(args)); \
	if (ret != 0) { \
		return ret; \
	} \
	return (int)(args).out;

int prover_init(void)
{
	/* First, let's set up a TEE-suitable heap */

	tee_free(g_tee_mempool, ATT_HEAP_SIZE);
	g_tee_mempool = tee_alloc(ATT_HEAP_SIZE);

	if (!g_tee_mempool) {
		return PRV_ERR_OOM;
	}

	g_tee_heap = mm_initialize("attestation_heap", g_tee_mempool, ATT_HEAP_SIZE);

	/* Invoke the actual prover_init() now */

	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_INIT);

	INVOKE(args);
}

void prover_destroy(void)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_DESTROY);

	tee_call(&args);

	/* Clean up our TEE mem pool (tee_free() wipes as well) */

	tee_free(g_tee_mempool, ATT_HEAP_SIZE);
	g_tee_mempool = NULL;
	g_tee_heap = NULL;
}

int prover_session_state(uint32_t session_id)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_SESSION_STATE);

	args.a[0].v = session_id;

	INVOKE(args);
}

int prover_session_init(uint32_t session_id, int server)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_SESSION_INIT);

	args.a[0].v = session_id;

	args.a[1].v = (uintptr_t)server;

	INVOKE(args);
}

void prover_session_close(uint32_t session_id, uint8_t *out, size_t *out_len)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_SESSION_CLOSE);

	args.a[0].v = session_id;

	args.a[1].p = out;
	args.a[1].is_ptr = 1;

	args.a[2].p = out_len;
	args.a[2].size = sizeof(size_t);

	tee_call(&args);
}

int prover_session_recv(uint32_t session_id, const uint8_t *in, size_t *in_len,
			uint8_t *out, size_t *out_len, uint8_t *appdata,
			size_t appdata_len)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_SESSION_RECV);

	args.a[0].v = session_id;

	args.a[1].p = (uint8_t *)in;
	args.a[1].is_ptr = 1;

	args.a[2].p = in_len;
	args.a[2].size = sizeof(size_t);

	args.a[3].p = out;
	args.a[3].is_ptr = 1;

	args.a[4].p = out_len;
	args.a[4].size = sizeof(size_t);

	args.a[5].p = appdata;
	args.a[5].size = appdata_len;

	INVOKE(args);
}

int prover_session_send(uint32_t session_id, const uint8_t *appdata,
			size_t appdata_len, uint8_t *out, size_t *out_len)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_SESSION_SEND);

	args.a[0].v = session_id;

	args.a[1].p = (uint8_t *)appdata;
	args.a[1].size = appdata_len;

	args.a[2].p = out;
	args.a[2].is_ptr = 1;

	args.a[3].p = out_len;
	args.a[3].size = sizeof(size_t);

	INVOKE(args);
}

int prover_challenge(const uint8_t *tlv_challenge, size_t chlen,
		     uint8_t *tlv_report, size_t relen)
{
	teeioc_call_t args = NEW_CALL(SBI_EXT_PRV_CHALLENGE);

	args.a[0].p = (uint8_t *)tlv_challenge;
	args.a[0].size = chlen;

	args.a[1].p = tlv_report;
	args.a[1].size = relen;

	INVOKE(args);
}



