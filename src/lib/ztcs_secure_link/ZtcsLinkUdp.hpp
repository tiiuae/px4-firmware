/****************************************************************************
 * A secure_udp::Udp backed by the Noise link the aircraft already has.
 *
 * The firmware updater talks to an interface, not to a socket, so carrying it
 * over the enrolled identity is a matter of implementing that interface. It
 * replaces a second key exchange, its own keys and its own enrolment with the
 * one already in the enclave.
 ****************************************************************************/

#pragma once

#include <secure_udp/SecureUdp.hpp>

#include "secure_link.h"

namespace ztcs
{

class ZtcsLinkUdp : public secure_udp::Udp
{
public:
	/* Borrowed, not owned. A shared link can be passed here later without
	 * this class changing.
	 */
	ZtcsLinkUdp(struct secure_link *link, const char *remote, uint16_t local_port,
		    uint16_t remote_port, unsigned timeout_ms);
	~ZtcsLinkUdp() override;

	bool init() override;
	bool open(uint16_t remote_port = 0) override;
	void close() override;

	ssize_t send(const void *buf, size_t len, int flags) override;
	ssize_t recv(void *buf, size_t len, int flags) override;
	ssize_t recvfrom(void *buf, size_t len, int flags, struct sockaddr *src_addr,
			 socklen_t *addrlen) override;

	/* The link rekeys on its own schedule, so these are not ours to drive. */
	void set_new_key_request(const char *prefix = nullptr) override;
	void invalidate_key_for(CryptoOp op) override;

	void print_stats() const override;
	size_t overhead_size() const override;
	const char *get_remote_address() const override { return _remote; }

private:
	/* Handshake and rekey datagrams are due whether or not there is
	 * traffic, so every send and receive gives the link a turn.
	 */
	void pump();

	struct secure_link *_link;
	char _remote[INET_ADDRSTRLEN] {};
	uint16_t _local_port;
	unsigned _timeout_ms;
	bool _owns_link{false};
	struct secure_link _own {};
};

} /* namespace ztcs */
