#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ztcs_link_udp;

struct ztcs_link_udp *ztcs_link_udp_open(const char *remote, uint16_t remote_port);
ssize_t ztcs_link_udp_send(struct ztcs_link_udp *link, const void *buf, size_t len);
ssize_t ztcs_link_udp_recv(struct ztcs_link_udp *link, void *buf, size_t len, unsigned timeout_ms);
void ztcs_link_udp_close(struct ztcs_link_udp *link);

#ifdef __cplusplus
}
#endif
