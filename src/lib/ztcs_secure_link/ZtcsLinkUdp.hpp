/****************************************************************************
 * A ztcs::Transport backed by the Noise link the aircraft is enrolled on.
 ****************************************************************************/

#pragma once

#include "ZtcsTransport.hpp"

#include <netinet/in.h>

#include "secure_link.h"

namespace ztcs
{

class ZtcsLinkUdp : public Transport
{
public:
	/* Borrowed: a shared link can be passed in later. */
	ZtcsLinkUdp(struct secure_link *link, const char *remote, uint16_t local_port,
		    uint16_t remote_port, unsigned timeout_s);
	~ZtcsLinkUdp() override;

	bool init() override;
	bool open(uint16_t remote_port = 0) override;
	void close() override;

	ssize_t send(const void *buf, size_t len, int flags) override;
	ssize_t recv(void *buf, size_t len, int flags) override;
	ssize_t recvfrom(void *buf, size_t len, int flags, struct sockaddr *src_addr,
			 socklen_t *addrlen) override;
	ssize_t recv_within(void *buf, size_t len, unsigned timeout_ms);

	bool set_timeout(unsigned seconds) override;
	void print_stats() const override;
	size_t overhead_size() const override;
	const char *remote_address() const override { return _remote; }
	uint16_t remote_port() const override { return _remote_port; }

private:
	void pump();
	bool establish();
	void set_timeout_ms(unsigned ms);

	/* Off the stack: the updater's task has 16k. */
	uint8_t _frame[1500];
	uint8_t _scratch[1500];

	uint32_t _rx{0};
	uint32_t _tx{0};

	struct secure_link *_link;
	int _sockfd{-1};
	uint16_t _remote_port{0};
	struct sockaddr_in _addr {};
	struct sockaddr_in _remote_addr {};
	char _remote[INET_ADDRSTRLEN] {};
	uint16_t _local_port;
	unsigned _timeout_s;  /* seconds, as the caller counts them */
	unsigned _handshake_timeout_ms{30000};
	bool _owns_link{false};
	struct secure_link _own {};
};

} /* namespace ztcs */
