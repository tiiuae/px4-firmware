#include "handshake.h"

int noise_hs_start(struct noise_hs *hs, const struct noise_static_key *s,
                   const uint8_t rs_pub[NOISE_DHLEN],
                   const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN],
                   uint8_t *out, size_t *out_len)
{
  return noise_initiator_start(&hs->ini, s, rs_pub, identity, out, out_len);
}

int noise_hs_finish(struct noise_hs *hs, const uint8_t *frame,
                    size_t frame_len, struct noise_session *out)
{
  return noise_initiator_finish(&hs->ini, frame, frame_len, out);
}

void noise_hs_end(struct noise_hs *hs)
{
  noise_wipe(&hs->ini, sizeof(hs->ini));
}
