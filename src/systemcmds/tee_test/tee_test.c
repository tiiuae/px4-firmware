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
/**
 * @file tee_test.c
 * Basic testing of TEE IOCTLs and attestation prover
 * Requires M-mode SBI support
 *
 * @author George Poulios
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <tee.h>
#include <nuttx/random.h>
#include <pfsoc/prover_ecall.h>
#include <prover/prover.h>
#include <libatt/attestation.h>
#include <libatt/tlv.h>
#include <ree/logging.h>

const tlv_tag_t DEF_MEAS[] = {ATT_TAG_M_VEN_DESC,
			      ATT_TAG_M_VEN_PROV_CA_SHA256,
			      ATT_TAG_M_VEN_PLAT_KEY_SHA256,
			      ATT_TAG_M_INST_AK_SHA256,
			      ATT_TAG_M_INST_DAC_SHA256,
			      ATT_TAG_M_INST_CFG_DESC,
			      ATT_TAG_M_INST_COMP_TYPE_DESC
			     };

static int create_challenge(tlv_t *chall, const tlv_tag_t *meas, int num_meas)
{
	int ret, i;
	tlv_len_t chall_cap;
	tlv_t *nonce;
	int meas_contain_comptype;

	ret = 0;

	if (!chall || chall->len < (TLV_HEADER_LEN + ATT_MIN_NONCE_SIZE +                                   \
				    num_meas * TLV_HEADER_LEN)) {
		return ATT_ERR_INV_ARG;
	}

	chall_cap = chall->len;
	chall->tag = ATT_TAG_CHALLENGE;
	chall->len = 0;

	ret = tlv_reserve_child(chall, chall_cap, ATT_TAG_NONCE, ATT_MIN_NONCE_SIZE,
				&nonce);

	if (ret != TLV_SIZE_W_LEN(ATT_MIN_NONCE_SIZE)) {
		printf("Failed to reserve nonce TLV in challenge with capacity %u, "
		       "length: %u: error %d\n",
		       chall_cap, chall->len, ret);
		return TLV_ERR_TOO_SHORT;
	}

	arc4random_buf(&nonce->val, nonce->len);

	/*
	 * Add the measurements
	 */

	ret = tlv_add_empty_children(chall, chall_cap, meas, num_meas);

	if (ret != num_meas) {
		printf("Failed to add measurement TLVs in challenge with capacity "
		       "%u, length: %u, added %d/%d\n",
		       chall_cap, chall->len, ret, num_meas);
		return TLV_ERR_TOO_SHORT;
	}

	// always request component type measurement because we are using it
	// to index the DAC; This could be perhaps implemented differently
	meas_contain_comptype = 0;

	for (i = 0; i < num_meas; i++) {
		if (meas[i] == ATT_TAG_M_INST_COMP_TYPE_DESC) {
			meas_contain_comptype = 1;
			break;
		}
	}

	if (!meas_contain_comptype) {
		ret = tlv_add_child_with_tag(chall, chall_cap,
					     ATT_TAG_M_INST_COMP_TYPE_DESC, NULL);

		if (ret < 0) {
			printf("Failed to add component type measurement TLV in challenge with "
			       "capacity %u, length: %u, error: %d\n",
			       chall_cap, chall->len, ret);
			return ret;
		}
	}

	return 0;
}

static int initialized = 0;

int tee_test_main(int argc, char *argv[])
{
	int ret;
	const tlv_tag_t *meas = DEF_MEAS;
	int num_meas = sizeof(DEF_MEAS) / sizeof(tlv_tag_t);
	tlv_t *challenge;
	tlv_t *report;

	printf("tee_test: prover_session_state(0): %d\n", prover_session_state(0));
	printf("tee_test: prover_init():  %d\n", prover_init());
	printf("tee_test: prover_session_state(0): %d\n", prover_session_state(0));
	printf("tee_test: prover_session_init(0, 1): %d\n", prover_session_init(0, 1));

	if (!initialized) {
		printf("tee_test: prover_session_state(0): %d\n", prover_session_state(0));
		printf("tee_test: prover_session_state(1): %d\n", prover_session_state(1));
		// printf("tee_test: prover_session_send(0,): %d\n", prover_session_send(0, (uint8_t *)str1, str1_sz, NULL, NULL));
		initialized = 1;
	}

	challenge = (tlv_t *)tee_alloc(ATT_MAX_CHALLENGE_LEN);

	if (!challenge) {
		printf("tee_test: tee_alloc(%ld) failed\n", ATT_MAX_CHALLENGE_LEN);
		return 1;
	}

	challenge->len = ATT_MAX_CHALLENGE_LEN - TLV_HEADER_LEN;

	if ((ret = create_challenge(challenge, meas, num_meas)) != 0) {
		printf("tee_test: create_challenge() error: %d\n", ret);
		return ret;
	}

	report = (tlv_t *)tee_alloc(ATT_MAX_REPORT_LEN);

	if (!report) {
		printf("tee_test: tee_alloc(%d) failed\n", ATT_MAX_REPORT_LEN);
		return 1;
	}


	// libatt_dump(challenge, "[challenge] ");

	printf("tee_test: prover_challenge(): %d\n", prover_challenge(
			(uint8_t *)challenge, ATT_MAX_CHALLENGE_LEN,
			(uint8_t *)report, ATT_MAX_REPORT_LEN));

	printf("tee_test: prover_session_init(0, 1): %d\n", prover_session_init(0, 1));

	// libatt_dump(report, "[report] ");

	// printf("tee_test: prover_destroy()\n");
	// prover_session_close(0, NULL, NULL);
	// prover_destroy();

	// TODO: I think this kind of shared memory is leaked if we exit abruptly
	//       without freeing. This is probably something the kernel should
	// 	 take care of. Either with a hook to process exit, or map it
	//       using an API other than vm_alloc_region() / up_shmat().

	if ((ret = tee_free(challenge, ATT_MAX_CHALLENGE_LEN)) != 0) {
		printf("tee_free(challenge): %d\n", ret);
	}

	if ((ret = tee_free(report, ATT_MAX_REPORT_LEN)) != 0) {
		printf("tee_free(report): %d\n", ret);
	}

	return PX4_OK;
}
