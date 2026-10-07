#include "ztcs_link_udp.h"
#include "ZtcsLinkUdp.hpp"

struct ztcs_link_udp : ztcs::ZtcsLinkUdp {
	using ZtcsLinkUdp::ZtcsLinkUdp;
};

extern "C" struct ztcs_link_udp *ztcs_link_udp_open(const char *remote, uint16_t remote_port)
{
	auto *link = new ztcs_link_udp(nullptr, remote, 0, remote_port, 5);

	if (link != nullptr && !link->open()) {
		delete link;
		return nullptr;
	}

	return link;
}

extern "C" ssize_t ztcs_link_udp_send(struct ztcs_link_udp *link, const void *buf, size_t len)
{
	return link->send(buf, len, 0);
}

extern "C" ssize_t ztcs_link_udp_recv(struct ztcs_link_udp *link, void *buf, size_t len, unsigned timeout_ms)
{
	return link->recv_within(buf, len, timeout_ms);
}

extern "C" void ztcs_link_udp_close(struct ztcs_link_udp *link)
{
	delete link;
}
