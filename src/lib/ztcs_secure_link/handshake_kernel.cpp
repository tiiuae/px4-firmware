#include "handshake.h"

#include <px4_platform/board_ctrl.h>
#include <px4_platform_common/crypto_backend.h>
#include <sys/boardctl.h>

#include <string.h>

extern "C" int noise_hs_start(struct noise_hs *hs, const struct noise_static_key *s,
			      const uint8_t rs_pub[NOISE_DHLEN],
			      const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN],
			      uint8_t *out, size_t *out_len)
{
	size_t n = NOISE_MSG1_LEN;
	cryptoiocnoisestart_t d = {s->index, rs_pub, identity, NOISE_IDENTITY_PAYLOAD_LEN, out, &n, NOISE_ERR_BACKEND};

	noise_hs_end(hs);

	if (boardctl(CRYPTOIOCNOISESTART, reinterpret_cast<unsigned long>(&d)) != 0 || d.handle <= 0) {
		return d.handle < 0 ? d.handle : NOISE_ERR_BACKEND;
	}

	hs->handle = d.handle;
	*out_len = n;
	return NOISE_OK;
}

extern "C" int noise_hs_finish(struct noise_hs *hs, const uint8_t *frame,
			       size_t frame_len, struct noise_session *out)
{
	uint8_t send = 0;
	uint8_t recv = 0;
	cryptoiocnoisefinish_t d = {hs->handle, frame, frame_len, &send, &recv, NOISE_ERR_STATE};

	if (hs->handle <= 0 || boardctl(CRYPTOIOCNOISEFINISH, reinterpret_cast<unsigned long>(&d)) != 0) {
		return NOISE_ERR_STATE;
	}

	if (d.ret != NOISE_OK) {
		return d.ret;
	}

	hs->handle = 0;
	memset(out, 0, sizeof(*out));

	if (noise_session_key_adopt(&out->send, send) != 0 || noise_session_key_adopt(&out->recv, recv) != 0) {
		noise_session_wipe(out);
		return NOISE_ERR_BACKEND;
	}

	return NOISE_OK;
}

extern "C" void noise_hs_end(struct noise_hs *hs)
{
	if (hs->handle > 0) {
		boardctl(CRYPTOIOCNOISEABORT, static_cast<unsigned long>(hs->handle));
	}

	hs->handle = 0;
}
