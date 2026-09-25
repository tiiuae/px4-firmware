#pragma once
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstddef>
enum CryptoOp { Encrypt = 1, Decrypt = 2 };
namespace secure_udp {
inline bool set_socket_timeout_option(int fd, unsigned ms)
{
	struct timeval tv { (time_t)(ms / 1000), (suseconds_t)((ms % 1000) * 1000) };
	return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
}
class Udp {
public:
	virtual ~Udp() = default;
	virtual bool init() = 0;
	virtual bool open(uint16_t remote_port = 0) = 0;
	virtual void close() = 0;
	virtual ssize_t send(const void *, size_t, int) = 0;
	virtual ssize_t recv(void *, size_t, int) = 0;
	virtual ssize_t recvfrom(void *, size_t, int, struct sockaddr *, socklen_t *) = 0;
	virtual void set_new_key_request(const char * = nullptr) = 0;
	virtual void invalidate_key_for(CryptoOp) = 0;
	virtual void print_stats() const = 0;
	virtual size_t overhead_size() const = 0;
	virtual const char *get_remote_address() const = 0;
	bool set_socket_timeout(unsigned ms) { return sockfd_ > 0 ? set_socket_timeout_option(sockfd_, ms) : false; }
protected:
	uint16_t remote_port_{0};
	int sockfd_{-1};
	struct sockaddr_in remote_addr_ {};
	struct sockaddr_in addr_ {};
};
}
