#include <unit_test.h>
#include <px4_platform_common/crypto_backend.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <unistd.h>

class IsolationTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool test_loads_fault();
	bool test_syscall_pointers_fault();
	bool test_nested_pointers_fault();
	bool test_ioctl_refused();
	bool test_crypto_refused();
	bool test_hrt_refused();
	bool test_spawn_refused();
};

static const struct {
	const char *name;
	uintptr_t addr;
	bool memory;
} targets[] {
	{"kernel", CONFIG_RAM_START, true},
	{"page pool", CONFIG_ARCH_PGPOOL_PBASE, true},
	{"session keys", 0x20499000, true},
	{"ELE mailbox", 0x47520000, false},
};

static const uintptr_t session_keys = targets[2].addr;

static int probe(const char *op, uintptr_t addr)
{
	char hex[19];
	char *const argv[] {(char *)"tests", (char *)"isolation", (char *)op, hex, nullptr};
	pid_t pid;
	int status = -1;

	snprintf(hex, sizeof(hex), "0x%lx", (unsigned long)addr);

	if (posix_spawnp(&pid, "tests", nullptr, nullptr, argv, nullptr) != 0 || waitpid(pid, &status, 0) != pid) {
		return -1;
	}

	return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

bool IsolationTest::test_loads_fault()
{
	int reached = 0;

	for (const auto &t : targets) {
		const bool killed = probe("load", t.addr) == SIGSEGV;
		PX4_INFO("load from %s: %s", t.name, killed ? "process killed" : "READ");
		reached += !killed;
	}

	ut_compare("loads that reached a protected address", reached, 0);
	return true;
}

bool IsolationTest::test_syscall_pointers_fault()
{
	int copied = 0;

	for (const auto &t : targets) {
		if (t.memory) {
			const bool killed = probe("write", t.addr) == SIGSEGV;
			PX4_INFO("write() from %s: %s", t.name, killed ? "process killed" : "COPIED");
			copied += !killed;
		}
	}

	ut_compare("syscalls that copied a protected address", copied, 0);
	return true;
}

bool IsolationTest::test_nested_pointers_fault()
{
	const bool killed = probe("writev", session_keys) == SIGSEGV;
	PX4_INFO("writev() of session keys: %s", killed ? "process killed" : "COPIED");
	ut_assert("a nested pointer reached a protected address", killed);
	return true;
}

bool IsolationTest::test_ioctl_refused()
{
	const int fd = open("/dev/null", O_RDONLY);
	ut_assert("open /dev/null", fd >= 0);

	errno = 0;
	const int ret = ioctl(fd, FIONREAD, (unsigned long)session_keys);
	const int err = errno;
	close(fd);

	PX4_INFO("ioctl() into session keys: %d, errno %d", ret, err);
	ut_assert("ioctl wrote to a protected address", ret < 0 && err == EFAULT);
	return true;
}

bool IsolationTest::test_crypto_refused()
{
	crypto_session_handle_t own{};
	cryptoiocopen_t open {CRYPTO_CHACHA20_POLY1305, &own};
	ut_compare("session opens", boardctl(CRYPTOIOCOPEN, (uintptr_t)&open), 0);
	ut_assert("session valid", crypto_session_handle_valid(own));

	crypto_session_handle_t forged = own;
	forged.handle = 1000;
	uint8_t cipher[16];
	size_t cipher_size = sizeof(cipher);
	const uint8_t message[16] {};

	cryptoiocencrypt_t foreign {&forged, 0, message, sizeof(message), cipher, &cipher_size, nullptr, nullptr, false};
	errno = 0;
	int ret = boardctl(CRYPTOIOCENCRYPT, (uintptr_t)&foreign);
	PX4_INFO("encrypt under a forged session: %d, errno %d", ret, errno);
	ut_assert("a forged session sealed", ret < 0 && errno == EFAULT && !foreign.ret);

	cryptoiocencrypt_t kernel {&own, 0, (const uint8_t *)session_keys, 16, cipher, &cipher_size, nullptr, nullptr, false};
	errno = 0;
	ret = boardctl(CRYPTOIOCENCRYPT, (uintptr_t)&kernel);
	PX4_INFO("encrypt of session keys: %d, errno %d", ret, errno);
	ut_assert("the session keys were sealed out", ret < 0 && errno == EFAULT && !kernel.ret);

	ut_compare("session closes", boardctl(CRYPTOIOCCLOSE, (uintptr_t)&own), 0);
	return true;
}

bool IsolationTest::test_hrt_refused()
{
	px4_hrt_handle_t forged = (px4_hrt_handle_t)targets[0].addr;

	errno = 0;
	const int ret = boardctl(HRT_UNREGISTER, (uintptr_t)&forged);
	PX4_INFO("hrt unregister of a forged handle: %d, errno %d", ret, errno);
	ut_assert("a forged hrt handle was freed", ret < 0 && errno == EFAULT);
	return true;
}

bool IsolationTest::test_spawn_refused()
{
	char *const argv[] {(char *)"tests", (char *)session_keys, nullptr};
	pid_t pid;

	const int ret = posix_spawnp(&pid, "tests", nullptr, nullptr, argv, nullptr);
	PX4_INFO("spawn with session keys as an argument: %d", ret);

	if (ret == 0) {
		waitpid(pid, nullptr, 0);
	}

	ut_assert("a kernel argument was copied into a new process", ret == EFAULT);
	return true;
}

bool IsolationTest::run_tests()
{
	ut_run_test(test_loads_fault);
	ut_run_test(test_syscall_pointers_fault);
	ut_run_test(test_nested_pointers_fault);
	ut_run_test(test_ioctl_refused);
	ut_run_test(test_crypto_refused);
	ut_run_test(test_hrt_refused);
	ut_run_test(test_spawn_refused);

	return (_tests_failed == 0);
}

extern "C" int test_isolation(int argc, char *argv[])
{
	if (argc == 3) {
		const uintptr_t addr = strtoul(argv[2], nullptr, 0);
		struct iovec iov {(void *)addr, 32};
		int fds[2];

		if (!strcmp(argv[1], "load")) {
			(void) * (volatile uint32_t *)addr;

		} else if (pipe(fds) == 0) {
			if (!strcmp(argv[1], "write")) {
				(void)write(fds[1], (const void *)addr, 32);

			} else {
				(void)writev(fds[1], &iov, 1);
			}
		}

		return 0;
	}

	IsolationTest *test = new IsolationTest();
	const bool success = test->run_tests();
	test->print_results();
	delete test;
	return success ? 0 : -1;
}
