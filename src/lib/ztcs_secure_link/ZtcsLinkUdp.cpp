/****************************************************************************
 * A secure_udp::Udp backed by the Noise link. See ZtcsLinkUdp.hpp.
 ****************************************************************************/

#include "ZtcsLinkUdp.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ztcs
{

/* One sealed datagram, plus what the link puts in front of and behind it. */
static constexpr size_t FRAME_MAX = 1500;

ZtcsLinkUdp::ZtcsLinkUdp(struct secure_link *link, const char *remote, uint16_t local_port,
			 uint16_t remote_port, unsigned timeout_ms)
	: _link(link), _local_port(local_port), _timeout_ms(timeout_ms)
{
	remote_port_ = remote_port;

	if (remote != nullptr) {
		strncpy(_remote, remote, sizeof(_remote) - 1);
	}
}

ZtcsLinkUdp::~ZtcsLinkUdp()
{
	close();
}

bool ZtcsLinkUdp::init()
{
	if (_link == nullptr) {
		struct secure_link_keys keys;

		if (!secure_link_ensure_keys(&keys)) {
			PX4_ERR("not enrolled, so there is no link to carry an update");
			return false;
		}

		if (secure_link_init(&_own, &keys, hrt_absolute_time()) != 0) {
			PX4_ERR("could not start a link");
			return false;
		}

		_link = &_own;
		_owns_link = true;
	}

	return true;
}

bool ZtcsLinkUdp::open(uint16_t remote_port)
{
	if (remote_port != 0) {
		remote_port_ = remote_port;
	}

	sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

	if (sockfd_ < 0) {
		PX4_ERR("socket: %d", errno);
		return false;
	}

	addr_.sin_family = AF_INET;
	addr_.sin_addr.s_addr = htonl(INADDR_ANY);
	addr_.sin_port = htons(_local_port);

	if (bind(sockfd_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
		PX4_ERR("bind %u: %d", _local_port, errno);
		close();
		return false;
	}

	remote_addr_.sin_family = AF_INET;
	remote_addr_.sin_port = htons(remote_port_);

	if (inet_pton(AF_INET, _remote, &remote_addr_.sin_addr) != 1) {
		PX4_ERR("remote address %s is not v4", _remote);
		close();
		return false;
	}

	set_socket_timeout(_timeout_ms);
	return true;
}

void ZtcsLinkUdp::close()
{
	if (sockfd_ >= 0) {
		::close(sockfd_);
		sockfd_ = -1;
	}
}

void ZtcsLinkUdp::pump()
{
	uint8_t frame[FRAME_MAX];
	const uint64_t now = hrt_absolute_time();

	int len = secure_link_poll(_link, now, frame, sizeof(frame));

	if (len > 0) {
		sendto(sockfd_, frame, len, 0, (struct sockaddr *)&remote_addr_,
		       sizeof(remote_addr_));
	}
}

ssize_t ZtcsLinkUdp::send(const void *buf, size_t len, int flags)
{
	uint8_t frame[FRAME_MAX];

	pump();

	int sealed = secure_link_seal(_link, hrt_absolute_time(),
				      (const uint8_t *)buf, len, frame, sizeof(frame));

	/* No session yet is not an error the updater can retry its way out of. */
	if (sealed < 0) {
		errno = ENOTCONN;
		return -1;
	}

	ssize_t sent = sendto(sockfd_, frame, sealed, flags,
			      (struct sockaddr *)&remote_addr_, sizeof(remote_addr_));

	return sent < 0 ? sent : (ssize_t)len;
}

ssize_t ZtcsLinkUdp::recvfrom(void *buf, size_t len, int flags, struct sockaddr *src_addr,
			      socklen_t *addrlen)
{
	uint8_t frame[FRAME_MAX];

	pump();

	ssize_t got = ::recvfrom(sockfd_, frame, sizeof(frame), flags, src_addr, addrlen);

	if (got <= 0) {
		return got;
	}

	int plain = secure_link_open(_link, hrt_absolute_time(), frame, got,
				     (uint8_t *)buf, len);

	/* Zero means the datagram was the link's own and is not the caller's to
	 * see; a handshake in flight must not look like a short read.
	 */
	if (plain <= 0) {
		errno = EAGAIN;
		return -1;
	}

	return plain;
}

ssize_t ZtcsLinkUdp::recv(void *buf, size_t len, int flags)
{
	return recvfrom(buf, len, flags, nullptr, nullptr);
}

void ZtcsLinkUdp::set_new_key_request(const char *prefix)
{
	(void)prefix;
}

void ZtcsLinkUdp::invalidate_key_for(CryptoOp op)
{
	(void)op;
}

void ZtcsLinkUdp::print_stats() const
{
	PX4_INFO("ztcs link: %s, %" PRIu32 " handshakes, %" PRIu32 " rejected",
		 _link->state == SECURE_LINK_ESTABLISHED ? "established" : "handshaking",
		 _link->handshakes, _link->decrypt_fails);
}

size_t ZtcsLinkUdp::overhead_size() const
{
	return NOISE_TRANSPORT_HDR_LEN + NOISE_TAGLEN;
}

} /* namespace ztcs */
