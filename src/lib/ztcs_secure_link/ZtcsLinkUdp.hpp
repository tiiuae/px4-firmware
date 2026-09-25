/****************************************************************************
 * A secure_udp::Udp backed by the Noise link the aircraft is enrolled on.
 ****************************************************************************/

#pragma once

#include <secure_udp/SecureUdp.hpp>

#include "secure_link.h"

namespace ztcs
{

class ZtcsLinkUdp : public secure_udp::Udp
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

	void set_new_key_request(const char *prefix = nullptr) override;
	void invalidate_key_for(CryptoOp op) override;

	void print_stats() const override;
	size_t overhead_size() const override;
	const char *get_remote_address() const override { return _remote; }

private:
	void pump();
	bool establish();

	/* Off the stack: the updater's task has 16k. */
	uint8_t _frame[1500];
	uint8_t _scratch[1500];

	struct secure_link *_link;
	char _remote[INET_ADDRSTRLEN] {};
	uint16_t _local_port;
	unsigned _timeout_s;  /* seconds, as the caller counts them */
	unsigned _handshake_timeout_ms{30000};
	bool _owns_link{false};
	struct secure_link _own {};
};

} /* namespace ztcs */
