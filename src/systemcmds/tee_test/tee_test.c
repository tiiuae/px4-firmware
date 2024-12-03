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
 * Basic testing of TEE IOCTLs; Requires M-mode SBI support
 *
 * @author George Poulios
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <tee.h>

#define SBI_EXT_PRV_INIT 		0x100

int tee_test_main(int argc, char *argv[])
{
#define str1_sz (1024)
#define huge_sz (16 * 1024)
	char *str1;
	char *huge;
	int ret;

	str1 = (char *)tee_alloc(str1_sz);
	memcpy(str1, "TEE buf test string", sizeof("TEE buf test string"));
	printf("tee_test: tee_alloc(str1): %p = %s\n", str1, str1);

	huge = (char *)tee_alloc(huge_sz);
	memset(huge, 0x41, huge_sz - 1);
	printf("tee_test: tee_alloc(huge): %p = [...]%s\n",
	       huge, huge + (huge_sz - 10));

	ret = tee_free(huge, huge_sz);
	printf("tee_test: tee_free(huge): %d\n", ret);

	huge = (char *)tee_alloc(huge_sz);
	memset(huge, 0x42, huge_sz - 1);
	printf("tee_test: tee_alloc(huge): %p = [...]%s\n",
	       huge, huge + (huge_sz - 10));

	teeioc_call_t args = {
		.funcid = SBI_EXT_PRV_INIT,
		.a = {
			{.p = str1, .size = str1_sz},
			{.p = huge, .size = huge_sz},
		},
		.out = 0
	};

	ret = tee_call(&args);

	printf("tee_test: tee_call(): %d, PRV_INIT result: %lu\n", ret, args.out);
	printf("  str1: %s\n  huge: [...]%s\n", str1, huge + (huge_sz - 10));

	return PX4_OK;
}
