/****************************************************************************
 * A ztcs::Transport backed by the Noise link. See ZtcsLinkUdp.hpp.
 ****************************************************************************/

#include "ZtcsLinkUdp.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ztcs
{

static constexpr size_t FRAME_MAX = 1500;

static constexpr unsigned HANDSHAKE_POLL_MS = 250;

ZtcsLinkUdp::ZtcsLinkUdp(struct secure_link *link, const char *remote, uint16_t local_port,
			 uint16_t remote_port, unsigned timeout_s)
	: _link(link), _local_port(local_port), _timeout_s(timeout_s)
{
	_remote_port = remote_port;

	if (remote != nullptr) {
		strncpy(_remote, remote, sizeof(_remote) - 1);
	}
}

ZtcsLinkUdp::~ZtcsLinkUdp()
{
	close();

	if (_owns_link) {
		secure_link_close(&_own);
	}
}

bool ZtcsLinkUdp::init()
{
	if (_link != nullptr) {
		return true;
	}

	{
		struct secure_link_keys keys;

		if (!secure_link_ensure_keys(&keys)) {
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
	if (!init()) {
		return false;
	}

	if (remote_port != 0) {
		_remote_port = remote_port;
	}

	_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_sockfd < 0) {
		PX4_ERR("socket: %d", errno);
		return false;
	}

	_addr.sin_family = AF_INET;
	_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	_addr.sin_port = htons(_local_port);

	int reuse = 1;
	setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	if (bind(_sockfd, (struct sockaddr *)&_addr, sizeof(_addr)) < 0) {
		PX4_ERR("bind %u: %d", _local_port, errno);
		close();
		return false;
	}

	struct sockaddr_in remote {};
	remote.sin_family = AF_INET;
	remote.sin_port = htons(_remote_port);

	if (inet_pton(AF_INET, _remote, &remote.sin_addr) != 1) {
		PX4_ERR("remote address %s is not v4", _remote);
		close();
		return false;
	}

	if (connect(_sockfd, (struct sockaddr *)&remote, sizeof(remote)) < 0) {
		PX4_ERR("connect %s:%u: %d", _remote, _remote_port, errno);
		close();
		return false;
	}

	set_timeout_ms(_timeout_s * 1000);

	return establish();
}

/* Nothing calls init(), and the first send needs a session. */
bool ZtcsLinkUdp::establish()
{
	if (_link == nullptr) {
		return false;
	}

	const uint64_t deadline = hrt_absolute_time() + (uint64_t)_handshake_timeout_ms * 1000;

	set_timeout_ms(HANDSHAKE_POLL_MS);

	while (hrt_absolute_time() < deadline) {
		if (_link->state == SECURE_LINK_ESTABLISHED) {
			set_timeout_ms(_timeout_s * 1000);
			return true;
		}

		pump();

		ssize_t got = ::recvfrom(_sockfd, _frame, sizeof(_frame), 0, nullptr, nullptr);

		if (got > 0) {
			secure_link_open(_link, hrt_absolute_time(), _frame, got, _scratch,
					 sizeof(_scratch));
		}
	}

	set_timeout_ms(_timeout_s * 1000);
	PX4_ERR("link did not come up in %ums", _handshake_timeout_ms);
	return false;
}

/* The base takes whole seconds; the handshake polls in fractions of one. */
void ZtcsLinkUdp::set_timeout_ms(unsigned ms)
{
	struct timeval tv {(time_t)(ms / 1000), (suseconds_t)((ms % 1000) * 1000)};
	setsockopt(_sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void ZtcsLinkUdp::close()
{
	if (_sockfd >= 0) {
		::close(_sockfd);
		_sockfd = -1;
	}
}

void ZtcsLinkUdp::pump()
{
	if (_link == nullptr) {
		return;
	}

	const uint64_t now = hrt_absolute_time();

	int len = secure_link_poll(_link, now, _frame, sizeof(_frame));

	if (len > 0) {
		::send(_sockfd, _frame, len, 0);
	}
}

ssize_t ZtcsLinkUdp::send(const void *buf, size_t len, int flags)
{
	if (_link == nullptr) {
		errno = ENOTCONN;
		return -1;
	}

	/* The station drops an idle session, so a send after silence starts a new one. */
	pump();

	int sealed = secure_link_seal(_link, hrt_absolute_time(),
				      (const uint8_t *)buf, len, _frame, sizeof(_frame));

	/* The caller never reopens, so a dropped session is rebuilt here or never. */
	if (sealed < 0 && establish()) {
		sealed = secure_link_seal(_link, hrt_absolute_time(),
					  (const uint8_t *)buf, len, _frame, sizeof(_frame));
	}

	if (sealed < 0) {
		errno = ENOTCONN;
		return -1;
	}

	ssize_t sent = ::send(_sockfd, _frame, sealed, flags);

	if (sent > 0) {
		_tx++;
	}

	return sent < 0 ? sent : (ssize_t)len;
}

ssize_t ZtcsLinkUdp::recvfrom(void *buf, size_t len, int flags, struct sockaddr *src_addr,
			      socklen_t *addrlen)
{
	if (_link == nullptr) {
		errno = ENOTCONN;
		return -1;
	}

	/* Station keepalives would reset a per datagram timeout forever. */
	const uint64_t deadline = hrt_absolute_time() + (uint64_t)_timeout_s * 1000000;

	while (hrt_absolute_time() < deadline) {
		ssize_t got = ::recvfrom(_sockfd, _frame, sizeof(_frame), flags, src_addr, addrlen);

		if (got <= 0) {
			print_stats();
			return got;
		}

		_rx++;

		int plain = secure_link_open(_link, hrt_absolute_time(), _frame, got,
					     (uint8_t *)buf, len);

		if (plain > 0) {
			return plain;
		}
	}

	print_stats();
	errno = EAGAIN;
	return -1;
}

ssize_t ZtcsLinkUdp::recv_within(void *buf, size_t len, unsigned timeout_ms)
{
	if (_link == nullptr) {
		errno = ENOTCONN;
		return -1;
	}

	const uint64_t deadline = hrt_absolute_time() + (uint64_t)timeout_ms * 1000;

	for (;;) {
		const uint64_t now = hrt_absolute_time();
		struct pollfd pfd {_sockfd, POLLIN, 0};
		int ready = ::poll(&pfd, 1, now < deadline ? (int)((deadline - now + 999) / 1000) : 0);

		if (ready <= 0) {
			return ready;
		}

		ssize_t got = ::recvfrom(_sockfd, _frame, sizeof(_frame), MSG_DONTWAIT, nullptr, nullptr);

		if (got <= 0) {
			return got;
		}

		_rx++;

		int plain = secure_link_open(_link, hrt_absolute_time(), _frame, got, (uint8_t *)buf, len);

		if (plain > 0) {
			return plain;
		}
	}
}

ssize_t ZtcsLinkUdp::recv(void *buf, size_t len, int flags)
{
	return recvfrom(buf, len, flags, nullptr, nullptr);
}

bool ZtcsLinkUdp::set_timeout(unsigned seconds)
{
	if (_sockfd < 0) {
		return false;
	}

	_timeout_s = seconds;
	set_timeout_ms(seconds * 1000);
	return true;
}

size_t ZtcsLinkUdp::overhead_size() const
{
	return NOISE_TRANSPORT_HDR_LEN + NOISE_TAGLEN;
}

void ZtcsLinkUdp::print_stats() const
{
	if (_link == nullptr) {
		PX4_INFO("ztcs link: not open");
		return;
	}

	PX4_INFO("ztcs link: %s, %" PRIu32 " hs, %" PRIu32 " rejected, tx %" PRIu32
		 ", rx %" PRIu32 ", peer %s:%u",
		 _link->state == SECURE_LINK_ESTABLISHED ? "established" : "handshaking",
		 _link->handshakes, _link->decrypt_fails, _tx, _rx, _remote,
		 (unsigned)_remote_port);
}

} /* namespace ztcs */
