/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <poll.h>
#include <termios.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/sensor_tvs.h>
#include <uORB/PublicationMulti.hpp>

#include <uORB/topics/telem_test_status.h>

using namespace time_literals;

#define MIN(a, b)					((a) <= (b) ? (a) : (b))

#define TELEM_UART_COUNT		4
#define LOOPBACK_TEST_DATA_LEN  50

#define STATUS_UNINITIALIZED		0x0
#define STATUS_IDLE					0x1
#define STATUS_RUNNING				0x2

#define STATUS_ACT_WRITE_ERR		0x4
#define STATUS_ACT_READ_ERR			0x8

#define STATUS_POLL_TIMEOUT			0x10
#define STATUS_POLL_ERR				0x20
#define STATUS_WRITE_ERR			0x40
#define STATUS_READ_ERR				0x80
#define STATUS_DATA_LEN_ERR			0x100
#define STATUS_DATA_ERR				0x200

static const hrt_abstime INFO_LOG_PERIOD = 30_s;

struct test_status {
	uint32_t status;
	uint32_t send;
	uint32_t recv;
	uint32_t err;
};

struct test_config {
	char dev_name[30];
	int fd;
	struct test_status test;
	ssize_t data_len;
	uint8_t *send_buffer;
	uint8_t *recv_buffer;
	hrt_abstime timestamp;
};

static struct test_config *test_ctx;

static uORB::Publication<telem_test_status_s> _status_pub{ORB_ID(telem_test_status)};

static int stop_test = 0;

static void app_sighandler(int sig_num)
{
	stop_test = 1;
}

static int setup_sigaction(void)
{
	struct sigaction act = {0};
	act.sa_handler = app_sighandler;

	sigaddset(&act.sa_mask, SIGTERM);
	if (sigaction(SIGTERM, &act, NULL) != 0) {
		PX4_ERR("sigaction1: errno=%d", errno);
		return 1;
	}

	return 0;
}

static void local_hexdump(void *mem, unsigned int len)
{
	for(unsigned int i = 0; i < len + ((len % 16) ? (16 - len % 16) : 0); i++)
	{
		/* print offset */
		if(i % 16 == 0)
		{
				printf("\n0x%06x: ", i);
		}

		/* print hex data */
		if(i < len)
		{
				printf("%02x ", 0xFF & ((char*)mem)[i]);
		}
		else /* end of block, just aligning for ASCII dump */
		{
				printf("   ");
		}
	}
	printf("\n");
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Telemetry UART test tool");
	PRINT_MODULE_USAGE_NAME("telem_test", "command");
	PRINT_MODULE_USAGE_PARAM_STRING('t', nullptr, nullptr, "Command name", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("flush", "Flush UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "Uart device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("loopback", "Loopback test between uarts");
	PRINT_MODULE_USAGE_ARG("<dev1>", "First UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev2>", "Second UART device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("uart_chain_loopback", "Loopback test for all uarts");
	PRINT_MODULE_USAGE_ARG("<dev1>", "First UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev2>", "Second UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev3>", "Third UART device", false);
	PRINT_MODULE_USAGE_ARG("<dev4>", "Fourth UART device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Send string to UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "Uart device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("recv", "Receive string from UART");
	PRINT_MODULE_USAGE_ARG("<dev>", "UART device", false);
}

static int set_port_param(int fd)
{
	struct termios uart_config = {0};

	if (tcgetattr(fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr");
		return 1;
	}

	if(cfsetspeed(&uart_config, 115200) < 0) {
			PX4_ERR("cfsetspeed");
			return 1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr");
		return 1;
	}

	return 0;
}

static int test_recv(int uart_recv, char *recv_buf, size_t buf_len)
{
	if (read(uart_recv, recv_buf, buf_len) < 0) {
		PX4_ERR("read: %d", errno);
		return 1;
	}

	return 0;
}

static int test_send(int uart_send, const char *send_buf)
{
	if (write(uart_send, send_buf, strlen(send_buf)) < 0) {
		PX4_ERR("write: %d", errno);
		return 1;
	}

	px4_usleep(200000);

	return 0;
}

static int loopback(int uart_send, int uart_recv, const char *send_buf)
{
	char recv_buf[15] = {0};

	if (test_send(uart_send, send_buf)) {
		return 1;
	}

	if (test_recv(uart_recv, recv_buf, sizeof(recv_buf) - 1)) {
		return 1;
	}

	if (strncmp(send_buf, recv_buf, strlen(send_buf)) != 0) {
		printf("ERROR: send != recv\n");
		printf("    send '%s'\n", send_buf);
		printf("    recv '%s'\n", recv_buf);
		return 1;
	}

	return 0;
}

static int test_loopback(int uart1, int uart2)
{
	const char *send_str1 = "send_uart_1";
	const char *send_str2 = "send_uart_2";

	printf("send: first -> second\n");

	if (loopback(uart1, uart2, send_str1)) {
		return 1;
	}

	printf("send: second -> first\n");

	if (loopback(uart2, uart1, send_str2)) {
		return 1;
	}

	return 0;
}

static int poll_uart(struct test_config *ctx, int timeout_ms, const char* err_msg, bool verbose)
{
	struct pollfd fds = {
		.fd = ctx->fd,
		.events = POLLIN,
	};

	int res = poll(&fds, 1, timeout_ms);

	if (res == 0) {
		if (verbose) PX4_ERR("%s poll: timeout (e:0x%x r:0x%x)", err_msg, fds.events, fds.revents);
		ctx->test.status |= STATUS_POLL_TIMEOUT;
		ctx->test.err++;
		return 1;
	}

	if (res < 0) {
		PX4_ERR("%s poll: %d (e:0x%x r:0x%x)", err_msg, errno, fds.events, fds.revents);
		ctx->test.status |= STATUS_POLL_ERR;
		ctx->test.err++;
		return 1;
	}

	if (!(fds.revents & POLLIN)) {
		PX4_ERR("%s poll: revents 0x%x", err_msg, fds.revents);
		ctx->test.status |= STATUS_POLL_ERR;
		ctx->test.err++;
		return 1;
	}

	return 0;
}

static void send_recv_verify(struct test_config *send_ctx, struct test_config *recv_ctx, bool verbose)
{
	ssize_t res = -1;
	ssize_t remaining = send_ctx->data_len;

	if (hrt_absolute_time() - send_ctx->timestamp > INFO_LOG_PERIOD) {
		send_ctx->timestamp = hrt_absolute_time();
		if (verbose) {
			PX4_INFO("%s -> %s: send %d, err %d (%d, %p)", send_ctx->dev_name, recv_ctx->dev_name,
				send_ctx->test.send, send_ctx->test.err, send_ctx->fd, send_ctx->send_buffer);

			PX4_INFO("%s -> %s: recv %d, err %d (%d, %p)", send_ctx->dev_name, recv_ctx->dev_name,
				recv_ctx->test.recv, recv_ctx->test.err, recv_ctx->fd, recv_ctx->recv_buffer);
		}
	}

	res = write(send_ctx->fd, send_ctx->send_buffer, send_ctx->data_len);

	if (res != send_ctx->data_len) {
		if (!(send_ctx->test.status & STATUS_ACT_WRITE_ERR)) {
			PX4_ERR("%s -> %s, write uart: %ld / %d", send_ctx->dev_name, recv_ctx->dev_name, res, errno);
		}

		send_ctx->test.status |= (STATUS_ACT_WRITE_ERR | STATUS_WRITE_ERR);
		send_ctx->test.err++;
		return;
	}

	send_ctx->test.send += send_ctx->data_len;

	send_ctx->test.status &= ~STATUS_ACT_WRITE_ERR;

	tcdrain(send_ctx->fd);

	while (remaining > 0) {
		if (poll_uart(recv_ctx, 1000, recv_ctx->dev_name, verbose)) {
			recv_ctx->test.status |= STATUS_ACT_READ_ERR;

			if (tcflush(recv_ctx->fd, TCIOFLUSH) < 0) {
				PX4_ERR("tcflush %s: %d", recv_ctx->dev_name, errno);
			}

			return;
		}

		res = read(recv_ctx->fd, recv_ctx->recv_buffer + (send_ctx->data_len - remaining), remaining);

		if (res < 0) {
			PX4_ERR("%s -> %s, uart read: %ld (%d)", send_ctx->dev_name, recv_ctx->dev_name, res, errno);
			recv_ctx->test.status |= (STATUS_ACT_READ_ERR | STATUS_READ_ERR);
			recv_ctx->test.err++;
			return;
		}

		remaining -= res;
	}

	if (remaining != 0) {
		if (!(send_ctx->test.status & STATUS_ACT_READ_ERR)) {
			PX4_ERR("%s -> %s, remaining != 0 (%ld)", send_ctx->dev_name, recv_ctx->dev_name, remaining);
		}
		recv_ctx->test.status |= (STATUS_ACT_READ_ERR | STATUS_DATA_LEN_ERR);
		recv_ctx->test.err++;
		return;
	}

	recv_ctx->test.recv += send_ctx->data_len;

	if (memcmp(send_ctx->send_buffer, recv_ctx->recv_buffer, send_ctx->data_len)) {
		if (!(send_ctx->test.status & STATUS_ACT_READ_ERR)) {
			PX4_ERR("%s -> %s, send != recv", send_ctx->dev_name, recv_ctx->dev_name);
		}

		if (verbose)
		{
			printf("send_buffer:\n");
			local_hexdump(send_ctx->send_buffer, send_ctx->data_len);
			printf("recv_buffer:\n");
			local_hexdump(recv_ctx->recv_buffer, send_ctx->data_len);
		}

		recv_ctx->test.status |= (STATUS_ACT_READ_ERR | STATUS_DATA_ERR);
		recv_ctx->test.err++;

		if (tcflush(recv_ctx->fd, TCIOFLUSH) < 0) {
			PX4_ERR("tcflush %s: %d", recv_ctx->dev_name, errno);
		}

		return;
	}

	recv_ctx->test.status &= ~STATUS_ACT_READ_ERR;
}

static void create_test_data()
{
	uint8_t counter = test_ctx[0].test.send % 207;

	for (int i = 0; i < TELEM_UART_COUNT; i++) {
		for (int j = 0; j < test_ctx[i].data_len; j++) {
			test_ctx[i].send_buffer[j] = counter % 256;
			counter++;
		}
	}
}

static void* run_uart_chain_loopback(void *args)
{
	struct test_config *ctx_tx = NULL;
	struct test_config *ctx_rx = NULL;
	bool verbose = *((bool*)args);

	PX4_INFO("thread started");

	for (int i = 0; i < TELEM_UART_COUNT; i++) {
		if (test_ctx[i].fd < 0) {
			PX4_ERR("%d. uart not specified", i + 1);
			stop_test = 1;
			return NULL;
		}

		if (set_port_param(test_ctx[i].fd)) {
			PX4_ERR("%s set_port_param", test_ctx[i].dev_name);
			stop_test = 1;
			return NULL;
		}

		if (tcflush(test_ctx[i].fd, TCIOFLUSH) < 0) {
			PX4_ERR("tcflush %s: %d", test_ctx[i].dev_name, errno);
			stop_test = 1;
			return NULL;
		}

		test_ctx[i].test.status = STATUS_RUNNING;
		test_ctx[i].data_len = LOOPBACK_TEST_DATA_LEN;
		test_ctx[i].send_buffer = (uint8_t*) calloc(1, LOOPBACK_TEST_DATA_LEN);
		test_ctx[i].recv_buffer = (uint8_t*) calloc(1, LOOPBACK_TEST_DATA_LEN);

		if (test_ctx[i].send_buffer == NULL || test_ctx[i].recv_buffer == NULL) {
			PX4_ERR("malloc");
			stop_test = 1;
			return NULL;
		}
	}

	while (!stop_test) {
		create_test_data();

		for (int tx_idx = 0; tx_idx < TELEM_UART_COUNT; tx_idx++) {
			int rx_idx = (tx_idx + 1) % TELEM_UART_COUNT;
			ctx_tx = &test_ctx[tx_idx];
			ctx_rx = &test_ctx[rx_idx];
			send_recv_verify(ctx_tx, ctx_rx, verbose);
		}
	}

	PX4_INFO("uart status:");

	for (int i = 0; i < TELEM_UART_COUNT; i++) {
		PX4_INFO("%s status 0x%x, send: %d, recv: %d, err:%d",
			test_ctx[i].dev_name, test_ctx[i].test.status, test_ctx[i].test.send,
			test_ctx[i].test.recv, test_ctx[i].test.err);

		test_ctx[i].test.status &= ~STATUS_RUNNING;
		test_ctx[i].test.status |= STATUS_IDLE;

		free(test_ctx[i].send_buffer);
		free(test_ctx[i].recv_buffer);
	}

	return NULL;
}

static void publish_telem_test_msg()
{
	telem_test_status_s report = {0};

	report.timestamp = hrt_absolute_time();
	for (int i = 0; i < TELEM_UART_COUNT; i++) {
		report.fd[i] = test_ctx[i].fd;
		report.status[i] = test_ctx[i].test.status;
		report.send[i] = test_ctx[i].test.send;
		report.recv[i] = test_ctx[i].test.recv;
		report.err[i] = test_ctx[i].test.err;
	}

	_status_pub.publish(report);
}

static int uart_chain_loopback(bool verbose)
{
	stop_test = 0;

	pthread_t loop_thread;
	pthread_attr_t th_attr;

	if (pthread_attr_init(&th_attr)) {
		PX4_ERR("pthread_attr_init");
		return 1;
	}

	if (pthread_attr_setstacksize(&th_attr, 4096)) {
		PX4_ERR("pthread_attr_setstacksize");
		return 1;
	}

	if (pthread_create(&loop_thread, &th_attr, run_uart_chain_loopback, &verbose))
	{
		PX4_ERR("pthread_create: %d", errno);
		return 1;
	}

	// Don't unadvertise after this. Leave the last topic data available.
	_status_pub.advertise();

	while(!stop_test) {
		publish_telem_test_msg();
		px4_sleep(1);
	}

	if (pthread_join(loop_thread, NULL))
	{
		PX4_ERR("pthread_join: %d", errno);
		return 1;
	}

	publish_telem_test_msg();

	return 0;

}

static void cleanup()
{
	struct test_config *ctx = NULL;

	for (int i = 0; i < TELEM_UART_COUNT; i++) {
		ctx = &test_ctx[i];

		if (ctx->fd < 1) continue;

		close(ctx->fd);
		ctx->fd = -1;
	}

	free(test_ctx);
	test_ctx = NULL;
}

static int parse_uarts(int arg_start, int arg_count, char *argv[])
{
	struct test_config *ctx = NULL;
	const char *uart_dev = NULL;
	int uart_count = MIN(TELEM_UART_COUNT, (arg_count - arg_start));

	for (int i = 0; i < TELEM_UART_COUNT; i++) {
		test_ctx[i].fd = -1;
	}

	for (int i = 0; i < uart_count; i++) {
		uart_dev = argv[arg_start + i];

		ctx = &test_ctx[i];

		ctx->fd = open(uart_dev, O_RDWR | O_NONBLOCK | O_NOCTTY);

		if (ctx->fd < 0) {
			PX4_ERR("%s open", uart_dev);
			cleanup();
			return 1;
		}

		strncpy(ctx->dev_name, uart_dev, sizeof(ctx->dev_name));

		PX4_INFO("%d. uart: %s, fd %d", i + 1, uart_dev, ctx->fd);
	}

	return 0;
}

extern "C" __EXPORT int telem_test_main(int argc, char *argv[])
{
	int res = 1;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	bool verbose = false;
	const char *test_type = nullptr;

	while ((ch = px4_getopt(argc, argv, "vt:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'v':
			printf("VERBOSE\n");
			verbose = true;
			break;

		case 't':
			test_type = myoptarg;
			printf("test_type: %s\n", test_type);
			break;

		default:
			PX4_ERR("Unknown option\n");
		}
	}

	if (test_type == nullptr) {
		usage(nullptr);
		return 1;
	}

	if (argc - myoptind < 1) {
		usage(nullptr);
		return 1;
	}

	test_ctx = (struct test_config *)calloc(TELEM_UART_COUNT, sizeof(struct test_config));
	if (!test_ctx) {
		PX4_ERR("failed to allocate ctx");
		return 1;
	}

	if (parse_uarts(myoptind, argc, argv)) {
		return 1;
	}

	printf("telem_test_main test type: %s\n", test_type);

	if (strcmp(test_type, "loopback") == 0) {
		if (test_ctx[1].fd < 0) {
			usage("ERROR: second uart not specified");
			cleanup();
			return 1;
		}

		res = test_loopback(test_ctx[0].fd, test_ctx[1].fd);

	} else if (strcmp(test_type, "uart_chain_loopback") == 0) {
		if (setup_sigaction()) {
			cleanup();
			return 1;
		}

		res = uart_chain_loopback(verbose);

	} else if (strcmp(test_type, "flush") == 0) {
		res = tcflush(test_ctx[0].fd, TCIOFLUSH);

		if (res < 0) {
			PX4_ERR("tcflush: %d", errno);
		}

	} else if (strcmp(test_type, "send") == 0) {
		const char *send_str = "send_uart";
		res = test_send(test_ctx[0].fd, send_str);

		printf("sent: '%s'\n", send_str);

	} else if (strcmp(test_type, "recv") == 0) {
		char recv_buf[20] = {0};
		res = test_recv(test_ctx[0].fd, recv_buf, sizeof(recv_buf) - 1);

		printf("recv: '%s'\n", recv_buf);
	} else if (strcmp(test_type, "set_port") == 0) {
		struct termios uart_config = {0};

		if (tcgetattr(test_ctx[0].fd, &uart_config) < 0) {
			PX4_ERR("tcgetattr");
			cleanup();
			return 1;
		}

		if(cfsetspeed(&uart_config, 115200) < 0) {
				PX4_ERR("cfsetspeed");
				cleanup();
				return 1;
		}

		if (tcsetattr(test_ctx[0].fd, TCSANOW, &uart_config) < 0) {
			PX4_ERR("tcsetattr");
			cleanup();
			return 1;
		}

	} else {
		PX4_ERR("test type not supported");
	}

	cleanup();

	PX4_INFO("telem_test EXIT\n");

	return res;
}
