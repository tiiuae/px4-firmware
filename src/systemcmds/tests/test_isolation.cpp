#include <unit_test.h>
#include <px4_platform_common/crypto_backend.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mmcsd.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spawn.h>
#include <nuttx/timers/pwm.h>
#include <arch/syscall.h>
#include <px4_platform/board_ctrl.h>

#if defined(CONFIG_LIB_ZTCS_SECURE_LINK)
#include <lib/ztcs_secure_link/noise/noise_ik.h>
#include <lib/ztcs_secure_link/secure_link_slots.h>
#endif
#include <sys/boardctl.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/syscall.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <time.h>
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
#if defined(CONFIG_LIB_ZTCS_SECURE_LINK)
	bool test_handshake_owned();
#endif
	bool test_hrt_refused();
	bool test_spawn_refused();
	bool test_environ_refused();
	bool test_anonymous_map();
	bool test_kernel_pointer_ioctls_refused();
	bool test_capabilities_enforced();
	bool test_bounds_refused();
	bool test_erase_bounded();
	bool test_spawn_race();
	bool test_nested_ioctls_refused();
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

#if defined(CONFIG_LIB_ZTCS_SECURE_LINK)
static int noise_start(int *handle)
{
	static const uint8_t station[NOISE_DHLEN] {9};
	static const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN] {};
	uint8_t msg[NOISE_MSG1_LEN];
	size_t len = sizeof(msg);
	cryptoiocnoisestart_t d {ZTCS_KEY_SLOT_LINK, station, identity, sizeof(identity), msg, &len, 0};
	const int ret = boardctl(CRYPTOIOCNOISESTART, (uintptr_t)&d);
	*handle = d.handle;
	return ret == 0 && d.handle > 0 && len == sizeof(msg) ? 0 : -1;
}

static int noise_finish(int handle)
{
	static const uint8_t reply[NOISE_MSG2_LEN] {NOISE_TYPE_HANDSHAKE_RESP};
	uint8_t send = 0;
	uint8_t recv = 0;
	cryptoiocnoisefinish_t d {handle, reply, sizeof(reply), &send, &recv, NOISE_OK};
	boardctl(CRYPTOIOCNOISEFINISH, (uintptr_t)&d);
	return d.ret;
}

bool IsolationTest::test_handshake_owned()
{
	int handle = 0;
	ut_compare("a handshake starts in the kernel", noise_start(&handle), 0);

	const int child = probe("noise", (uintptr_t)handle);
	PX4_INFO("another process on the handshake: %s", child == 0 ? "refused" : "REACHED");
	ut_compare("another process reached the handshake", child, 0);

	const int rc = noise_finish(handle);
	PX4_INFO("the owner's forged reply: %d", rc);
	ut_assert("the handshake survived the other process", rc != NOISE_ERR_STATE && rc != NOISE_OK);

	boardctl(CRYPTOIOCNOISEABORT, (uintptr_t)handle);
	ut_compare("an aborted handshake is gone", noise_finish(handle), NOISE_ERR_STATE);

	crypto_session_handle_t kex{};
	cryptoiocopen_t open {CRYPTO_X25519, &kex};
	boardctl(CRYPTOIOCOPEN, (uintptr_t)&open);
	static const uint8_t peer[NOISE_DHLEN] {9};
	uint8_t secret[NOISE_DHLEN] {};
	size_t secret_size = sizeof(secret);
	cryptoiockeyagreement_t k {&kex, ZTCS_KEY_SLOT_LINK, peer, sizeof(peer), secret, &secret_size, 0};
	const int raw = boardctl(CRYPTOIOCKEYAGREEMENT, (uintptr_t)&k);
	boardctl(CRYPTOIOCCLOSE, (uintptr_t)&kex);
	PX4_INFO("the link key's raw DH: %s", raw != 0 && !k.ret ? "refused" : "RETURNED");
	ut_assert("the link key's raw DH stays in the kernel", raw != 0 && !k.ret);
	return true;
}
#endif

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

bool IsolationTest::test_environ_refused()
{
	char **saved = get_environ_ptr();
	char *forged[] {(char *)session_keys, nullptr};
	char *const argv[] {(char *)"tests", nullptr};
	pid_t pid;

	ut_compare("setenv", setenv("ISOLATION", "1", 1), 0);
	ut_assert("getenv", getenv("ISOLATION") != nullptr && strcmp(getenv("ISOLATION"), "1") == 0);

	set_environ_ptr(forged);
	const int ret = posix_spawn(&pid, "/bin/tests", nullptr, nullptr, argv, nullptr);
	set_environ_ptr(saved);

	if (ret == 0) {
		waitpid(pid, nullptr, 0);
	}

	PX4_INFO("spawn inheriting session keys as environment: %d", ret);
	ut_assert("a kernel string was copied into a new environment", ret == EFAULT);
	ut_compare("unsetenv", unsetenv("ISOLATION"), 0);
	ut_assert("environment intact", getenv("ISOLATION") == nullptr);
	return true;
}

bool IsolationTest::test_anonymous_map()
{
	const size_t size = 3 * 4096;
	uint8_t *map = (uint8_t *)mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ut_assert("anonymous map", map != MAP_FAILED);

	bool zero = true;

	for (size_t i = 0; i < size; i++) {
		zero = zero && map[i] == 0;
	}

	PX4_INFO("anonymous map at %p, zeroed: %d", map, zero);
	ut_assert("anonymous pages are zeroed", zero);
	ut_assert("anonymous pages are user memory", (uintptr_t)map >= 0xC0000000);
	memset(map, 0xa5, size);
	ut_compare("munmap", munmap(map, size), 0);
	return true;
}

bool IsolationTest::test_kernel_pointer_ioctls_refused()
{
	const int fd = open("/dev/ram0", O_RDONLY);
	ut_assert("open /dev/ram0", fd >= 0);

	void *ptr = nullptr;
	errno = 0;
	const int xip = ioctl(fd, BIOC_XIPBASE, (unsigned long)&ptr);
	const int xip_err = errno;
	errno = 0;
	const int priv = ioctl(fd, DIOC_GETPRIV, (unsigned long)&ptr);
	const int priv_err = errno;
	close(fd);

	PX4_INFO("BIOC_XIPBASE: %d errno %d, DIOC_GETPRIV: %d errno %d, pointer %p", xip, xip_err, priv, priv_err, ptr);
	ut_assert("a kernel pointer was handed out", xip < 0 && xip_err == EPERM && priv < 0 && priv_err == EPERM
		  && ptr == nullptr);
	return true;
}

static int capability_probe()
{
	char *const argv[] {(char *)"ver", nullptr};
	platformioclaunch_t launch {1, (char **)argv, 0};
	pid_t pid;
	int got = 0;

	prctl(PR_CAPS_DROP, PR_CAP_ALL);

	const int fd = open("/dev/mtd_certs", O_RDONLY);
	got |= (fd >= 0 || errno != EPERM) << 0;

	if (fd >= 0) {
		close(fd);
	}

	got |= (posix_spawn(&pid, "/bin/tests", nullptr, nullptr, argv, nullptr) != EPERM) << 1;
	got |= (boardctl(PLATFORMIOCLAUNCH, (uintptr_t)&launch) == 0 || errno != EPERM) << 2;
	got |= (mount(nullptr, "/tmp/caps", "tmpfs", 0, nullptr) == 0 || errno != EPERM) << 3;
	got |= (prctl(PR_CAPS_GET) != 0) << 4;

	const pid_t parent = getppid();
	sched_param param {};
	sigevent event {};
	timer_t timer;
	event.sigev_notify = SIGEV_SIGNAL | SIGEV_THREAD_ID;
	event.sigev_signo = SIGUSR2;
	event.sigev_notify_thread_id = parent;

	got |= (kill(parent, SIGUSR2) == 0 || errno != EPERM) << 5;
	got |= (kill(parent, 0) != 0) << 6;
	got |= (sched_getparam(parent, &param) != 0 || sched_setparam(parent, &param) == 0 || errno != EPERM) << 7;
	got |= (timer_create(CLOCK_MONOTONIC, &event, &timer) == 0 || errno != EINVAL) << 8;
	got |= (sched_getparam(0, &param) != 0 || sched_setparam(getpid(), &param) != 0) << 9;
	return got;
}

bool IsolationTest::test_capabilities_enforced()
{
	ut_compare("tests holds every capability", prctl(PR_CAPS_GET), PR_CAP_ALL);

	signal(SIGUSR2, SIG_IGN);
	const int got = probe("caps", 0);
	signal(SIGUSR2, SIG_DFL);
	PX4_INFO("without capabilities, got through: 0x%x", got);
	ut_compare("a process without capabilities got through", got, 0);
	return true;
}

static int pwm_probe(const char *dev)
{
	pwm_info_s info {};
	info.frequency = 50;

	for (auto &channel : info.channels) {
		channel.channel = -1;
	}

	info.channels[0].channel = -2;
	info.channels[0].duty = 0x8000;

	const int fd = open(dev, O_RDONLY);

	if (fd < 0) {
		return -1;
	}

	int ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);

	if (ret == 0) {
		ret = ioctl(fd, PWMIOC_START, 0);
	}

	const int err = ret < 0 ? errno : 0;
	ioctl(fd, PWMIOC_STOP, 0);
	close(fd);
	return err;
}

bool IsolationTest::test_bounds_refused()
{
	const int nr = probe("syscall", 0);
	const int reserved = probe("reserved", 0);
	const int pages = probe("pgalloc", 0x100000000);
	const int flexio = pwm_probe("/dev/pwm1");
	const int tpm = pwm_probe("/dev/pwm_buzz");

	PX4_INFO("syscall one past the table: %s", nr == 0 ? "ENOSYS" : "DISPATCHED");
	PX4_INFO("reserved syscalls from user space: %s", reserved == 0 ? "ENOSYS" : "DISPATCHED");
	PX4_INFO("pgalloc above user space: %s", pages == 0 ? "refused" : "MAPPED");
	PX4_INFO("PWM channel -2: FlexIO errno %d, TPM errno %d", flexio, tpm);
	ut_assert("a bound let a caller through", nr == 0 && reserved == 0 && pages == 0 && flexio == EINVAL && tpm == EINVAL);
	return true;
}

bool IsolationTest::test_erase_bounded()
{
	mtd_geometry_s geo {};
	const int fd = open("/dev/mtd_px4_1", O_RDWR);
	ut_assert("open /dev/mtd_px4_1", fd >= 0);
	ut_compare("geometry", ioctl(fd, MTDIOC_GEOMETRY, (unsigned long)&geo), 0);

	mtd_erase_s erase {geo.neraseblocks, 1};
	errno = 0;
	const int ret = ioctl(fd, MTDIOC_ERASESECTORS, (unsigned long)&erase);
	const int err = errno;
	close(fd);

	PX4_INFO("erase block %u of a %u-block partition: %d, errno %d", (unsigned)erase.startblock,
		 (unsigned)geo.neraseblocks, ret, err);
	ut_assert("an erase left its partition", ret < 0 && err == ENXIO);
	return true;
}

static volatile bool racing;
static volatile unsigned long flips;
static char race_arg[4096];
static volatile char *race_path;

static void *race_flip(void *)
{
	volatile char *arg = race_arg;

	while (racing) {
		arg[1] = '\0';
		race_path[1] = '\0';
		arg[1] = 'A';
		race_path[1] = 'A';
		flips++;
	}

	return nullptr;
}

bool IsolationTest::test_spawn_race()
{
	char *const argv[] {(char *)"nsh", race_arg, nullptr};
	posix_spawn_file_actions_t actions;
	pthread_t flipper;
	int spawned = 0;

	pthread_attr_t attr;
	sched_param param {sched_get_priority_max(SCHED_FIFO)};
	cpu_set_t cpu;
	int enoent = 0;

	memset(race_arg, 'A', sizeof(race_arg) - 1);
	posix_spawn_file_actions_init(&actions);
	posix_spawn_file_actions_addopen(&actions, 1, race_arg, O_RDONLY, 0);
	race_path = ((spawn_open_file_action_s *)actions)->path;
	racing = true;

	CPU_ZERO(&cpu);
	CPU_SET(0, &cpu);
	sched_setaffinity(0, sizeof(cpu), &cpu);
	CPU_ZERO(&cpu);
	CPU_SET(1, &cpu);
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setschedparam(&attr, &param);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu), &cpu);
	const int created = pthread_create(&flipper, &attr, race_flip, nullptr);
	pthread_attr_destroy(&attr);
	ut_compare("flipper started", created, 0);

	for (int i = 0; i < 2000; i++) {
		pid_t pid;
		const int ret = posix_spawn(&pid, "/bin/nsh", &actions, nullptr, argv, nullptr);

		if (ret == 0) {
			waitpid(pid, nullptr, 0);
			spawned++;

		} else {
			enoent += ret == ENOENT || ret == ENAMETOOLONG;
		}
	}

	racing = false;
	pthread_join(flipper, nullptr);
	posix_spawn_file_actions_destroy(&actions);

	PX4_INFO("2000 spawns racing %lu flips of argv and file action lengths: kernel intact, %d reached the file action, %d started",
		 flips, enoent, spawned);
	ut_assert("the lengths never changed", flips > 0);
	ut_compare("a spawn opened a path that does not exist", spawned, 0);
	return true;
}

bool IsolationTest::test_nested_ioctls_refused()
{
	struct ifreq reqs[4];
	struct ifconf ifc {sizeof(reqs), {(char *)session_keys}};
	const int sock = socket(AF_INET, SOCK_DGRAM, 0);
	ut_assert("socket", sock >= 0);

	errno = 0;
	const int ifconf = ioctl(sock, SIOCGIFCONF, (unsigned long)&ifc);
	const int ifconf_err = errno;
	ifc = {sizeof(reqs), {(char *)reqs}};
	const int ifconf_own = ioctl(sock, SIOCGIFCONF, (unsigned long)&ifc);
	close(sock);

	uint8_t cid[512];
	mmc_ioc_cmd cmd {};
	cmd.opcode = 2;
	cmd.data_ptr = session_keys;
	const int fd = open("/dev/mmcsd0", O_RDONLY);
	ut_assert("open /dev/mmcsd0", fd >= 0);

	errno = 0;
	const int mmc = ioctl(fd, MMC_IOC_CMD, (unsigned long)&cmd);
	const int mmc_err = errno;
	cmd.data_ptr = (uintptr_t)cid;
	errno = 0;
	const int mmc_own = ioctl(fd, MMC_IOC_CMD, (unsigned long)&cmd);
	const int mmc_own_err = errno;
	close(fd);

	PX4_INFO("SIOCGIFCONF into session keys: %d errno %d, into its own buffer: %d, %u bytes",
		 ifconf, ifconf_err, ifconf_own, (unsigned)ifc.ifc_len);
	PX4_INFO("MMC_IOC_CMD into session keys: %d errno %d, into its own buffer: %d errno %d", mmc, mmc_err, mmc_own,
		 mmc_own_err);
	ut_assert("a nested pointer reached kernel memory", ifconf < 0 && ifconf_err == EFAULT && mmc < 0
		  && mmc_err == EFAULT);
	ut_assert("a legitimate nested pointer was refused", ifconf_own == 0 && (mmc_own == 0 || mmc_own_err != EFAULT));
	return true;
}

bool IsolationTest::run_tests()
{
	ut_run_test(test_loads_fault);
	ut_run_test(test_syscall_pointers_fault);
	ut_run_test(test_nested_pointers_fault);
	ut_run_test(test_ioctl_refused);
	ut_run_test(test_crypto_refused);
#if defined(CONFIG_LIB_ZTCS_SECURE_LINK)
	ut_run_test(test_handshake_owned);
#endif
	ut_run_test(test_hrt_refused);
	ut_run_test(test_spawn_refused);
	ut_run_test(test_environ_refused);
	ut_run_test(test_anonymous_map);
	ut_run_test(test_kernel_pointer_ioctls_refused);
	ut_run_test(test_capabilities_enforced);
	ut_run_test(test_erase_bounded);
	ut_run_test(test_spawn_race);
	ut_run_test(test_nested_ioctls_refused);
	ut_run_test(test_bounds_refused);

	return (_tests_failed == 0);
}

extern "C" int test_isolation(int argc, char *argv[])
{
	if (argc == 3) {
		const uintptr_t addr = strtoul(argv[2], nullptr, 0);
		struct iovec iov {(void *)addr, 32};
		int fds[2];

		if (!strcmp(argv[1], "caps")) {
			return capability_probe();

		} else if (!strcmp(argv[1], "syscall")) {
			return (long)sys_call0(SYS_maxsyscall) != -ENOSYS;

		} else if (!strcmp(argv[1], "reserved")) {
			return (long)sys_call0(SYS_switch_context) != -ENOSYS || (long)sys_call0(SYS_signal_handler_return) != -ENOSYS;

		} else if (!strcmp(argv[1], "pgalloc")) {
			return sys_call2(SYS_pgalloc, addr, 1) != 0;

#if defined(CONFIG_LIB_ZTCS_SECURE_LINK)

		} else if (!strcmp(argv[1], "noise")) {
			boardctl(CRYPTOIOCNOISEABORT, addr);
			return noise_finish((int)addr) != NOISE_ERR_STATE;
#endif

		} else if (!strcmp(argv[1], "load")) {
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
