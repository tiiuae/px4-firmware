/* Drives ZtcsLinkUdp against a real socket and a peer that answers on its own
 * schedule. The failures worth catching here are the ones a mock cannot show:
 * a loop that never yields, and a handshake nobody drives.
 */
#include "ZtcsLinkUdp.hpp"

#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <thread>
#include <atomic>
#include <chrono>

using namespace std::chrono;

static std::atomic<bool> stop_peer{false};
static std::atomic<int> peer_rx{0};

/* The station: answers a handshake datagram with one of its own, and echoes
 * payload back sealed.
 */
static void peer(uint16_t port, bool answer)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(port);
	a.sin_addr.s_addr = inet_addr("127.0.0.1");
	bind(fd, (sockaddr *)&a, sizeof(a));
	timeval tv{0, 50000}; setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	uint8_t buf[2048];
	while (!stop_peer) {
		sockaddr_in from{}; socklen_t fl = sizeof(from);
		ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, (sockaddr *)&from, &fl);
		if (n <= 0) { continue; }
		peer_rx++;
		if (!answer) { continue; }
		sendto(fd, buf, n, 0, (sockaddr *)&from, fl);
	}
	close(fd);
}

static int failures = 0;

static void check(bool ok, const char *what)
{
	printf("%-52s %s\n", what, ok ? "ok" : "FAIL");
	if (!ok) { failures++; }
}

int main()
{
	/* 1. open() establishes, and does not spin doing it. */
	{
		stop_peer = false; peer_rx = 0;
		std::thread t(peer, 19010, true);
		std::this_thread::sleep_for(milliseconds(50));

		ztcs::ZtcsLinkUdp u(nullptr, "127.0.0.1", 19011, 19010, 5);
		auto t0 = steady_clock::now();
		bool up = u.open();
		auto ms = duration_cast<milliseconds>(steady_clock::now() - t0).count();

		check(up, "open() brings the link up");
		check(ms < 2000, "open() returns promptly");
		u.close();
		stop_peer = true; t.join();
	}

	/* 2. A peer that never answers must time out, not spin forever. */
	{
		stop_peer = false; peer_rx = 0;
		std::thread t(peer, 19020, false);
		std::this_thread::sleep_for(milliseconds(50));

		ztcs::ZtcsLinkUdp u(nullptr, "127.0.0.1", 19021, 19020, 5);
		auto t0 = steady_clock::now();
		bool up = u.open();
		auto ms = duration_cast<milliseconds>(steady_clock::now() - t0).count();

		check(!up, "a silent peer fails rather than hanging");
		check(ms >= 900 && ms < 40000, "the failure respects a deadline");
		/* A spin sends as fast as the CPU allows; a paced loop does not. */
		check(peer_rx > 10 && peer_rx < 400,
		      "a silent peer is retried, and is not a send storm");
		printf("     datagrams sent while waiting: %d in %ldms\n", peer_rx.load(), ms);
		u.close();
		stop_peer = true; t.join();
	}

	/* 3. Payload survives a round trip once up. */
	{
		stop_peer = false; peer_rx = 0;
		std::thread t(peer, 19030, true);
		std::this_thread::sleep_for(milliseconds(50));

		ztcs::ZtcsLinkUdp u(nullptr, "127.0.0.1", 19031, 19030, 5);
		bool up = u.open();
		check(up, "link up for the payload case");

		const char *msg = "FW_UPDATE_REQ";
		ssize_t sent = u.send(msg, strlen(msg), 0);
		check(sent == (ssize_t)strlen(msg), "send reports the plaintext length");

		char back[256] = {};
		ssize_t got = u.recv(back, sizeof(back), 0);
		check(got == (ssize_t)strlen(msg) && memcmp(back, msg, got) == 0,
		      "recv returns the payload, not a handshake frame");

		check(u.overhead_size() == NOISE_TRANSPORT_HDR_LEN + NOISE_TAGLEN,
		      "overhead matches what the caller budgets for");
		u.close();
		stop_peer = true; t.join();
	}

	/* 4. The units the caller actually passes. The client's timeout is in
	 * seconds, and treating it as milliseconds paces the wait 1000x too
	 * fast, which on a priority task starves everything below it.
	 */
	{
		stop_peer = false; peer_rx = 0;
		std::thread t(peer, 19040, false);
		std::this_thread::sleep_for(milliseconds(50));

		/* 5 is what the updater passes: five seconds. */
		ztcs::ZtcsLinkUdp u(nullptr, "127.0.0.1", 19041, 19040, 5);
		auto t0 = steady_clock::now();
		u.open();
		auto ms = duration_cast<milliseconds>(steady_clock::now() - t0).count();
		double rate = peer_rx.load() * 1000.0 / (double)(ms ? ms : 1);

		printf("     send rate while waiting: %.1f/s\n", rate);
		check(rate < 50.0, "a 5 second timeout is not read as 5 milliseconds");
		u.close();
		stop_peer = true; t.join();
	}

	/* 5. The caller never calls init(). Setup belongs in open(), and
	 * nothing may dereference the link before it exists.
	 */
	{
		ztcs::ZtcsLinkUdp u(nullptr, "127.0.0.1", 19051, 19050, 5);
		char buf[16];
		check(u.send("x", 1, 0) < 0, "send before open fails rather than crashing");
		check(u.recv(buf, sizeof(buf), 0) < 0, "recv before open fails rather than crashing");
		u.print_stats();
		check(true, "print_stats before open does not crash");
	}

	printf("\n%s\n", failures ? "FAILURES" : "all ok");
	return failures ? 1 : 0;
}
