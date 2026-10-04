/****************************************************************************
 * A datagram transport carried by the Noise link, as the updater and the
 * DDS client see it.
 ****************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>

namespace ztcs
{

class Transport
{
public:
	virtual ~Transport() = default;

	virtual bool init() = 0;
	virtual bool open(uint16_t remote_port = 0) = 0;
	virtual void close() = 0;

	virtual ssize_t send(const void *buf, size_t len, int flags) = 0;
	virtual ssize_t recv(void *buf, size_t len, int flags) = 0;
	virtual ssize_t recvfrom(void *buf, size_t len, int flags, struct sockaddr *src_addr,
				 socklen_t *addrlen) = 0;

	virtual bool set_timeout(unsigned seconds) = 0;
	virtual void print_stats() const = 0;
	virtual size_t overhead_size() const = 0;
	virtual const char *remote_address() const = 0;
	virtual uint16_t remote_port() const = 0;
};

} /* namespace ztcs */
