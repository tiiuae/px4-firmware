#include <unit_test.h>
#include <px4_platform_common/log.h>

#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

class IsolationTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool test_loads_fault();
	bool test_syscalls_refuse();

	struct Target {
		const char *name;
		uintptr_t addr;
		bool memory;
	};

	static constexpr Target _targets[] {
		{"kernel", CONFIG_RAM_START, true},
		{"page pool", CONFIG_ARCH_PGPOOL_PBASE, true},
		{"session keys", 0x20499000, true},
		{"ELE mailbox", 0x47520000, false},
	};
};

bool IsolationTest::test_loads_fault()
{
	int reached = 0;

	for (const Target &t : _targets) {
		char addr[19];
		char *const argv[] {(char *)"tests", (char *)"isolation", addr, nullptr};
		pid_t pid;
		int status = -1;

		snprintf(addr, sizeof(addr), "0x%lx", (unsigned long)t.addr);

		if (posix_spawnp(&pid, "tests", nullptr, nullptr, argv, nullptr) != 0 || waitpid(pid, &status, 0) != pid) {
			PX4_ERR("probe of %s did not run", t.name);
			reached++;
			continue;
		}

		const bool faulted = WIFEXITED(status) && WEXITSTATUS(status) == SIGSEGV;
		PX4_INFO("load from %s: %s", t.name, faulted ? "faulted, process killed" : "READ");
		reached += !faulted;
	}

	ut_compare("user loads that reached a protected address", reached, 0);
	return true;
}

bool IsolationTest::test_syscalls_refuse()
{
	int copied = 0;
	int fds[2];

	ut_compare("pipe", pipe(fds), 0);

	for (const Target &t : _targets) {
		if (!t.memory) {
			continue;
		}

		errno = 0;
		const ssize_t n = write(fds[1], (const void *)t.addr, 32);
		PX4_INFO("write() from %s: %zd, errno %d", t.name, n, errno);

		if (n > 0) {
			char sink[32];
			copied++;
			read(fds[0], sink, sizeof(sink));
		}
	}

	close(fds[0]);
	close(fds[1]);

	ut_compare("syscalls that copied from a protected address", copied, 0);
	return true;
}

bool IsolationTest::run_tests()
{
	ut_run_test(test_loads_fault);
	ut_run_test(test_syscalls_refuse);

	return (_tests_failed == 0);
}

extern "C" int test_isolation(int argc, char *argv[])
{
	if (argc == 2) {
		(void) * (volatile uint32_t *)strtoul(argv[1], nullptr, 0);
		return 0;
	}

	IsolationTest *test = new IsolationTest();
	const bool success = test->run_tests();
	test->print_results();
	delete test;
	return success ? 0 : -1;
}
