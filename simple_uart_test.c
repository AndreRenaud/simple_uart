#include "acutest.h"
#include "simple_uart.h"

#define UART_LOOPBACK_1 "/tmp/ttyS0"
#define UART_LOOPBACK_2 "/tmp/ttyS1"

static pid_t spawn_process(char * const args[])
{
	pid_t pid;
	pid = fork();

	TEST_ASSERT(pid >= 0);

	if (pid == 0) {
		execvp(args[0], args);
	}
	/* Give it time to start */
	usleep(100000);
	return pid;
}


static pid_t setup_loopback(void)
{
	char * const args[] = {"socat", "PTY,link=/tmp/ttyS0", "PTY,link=/tmp/ttyS1", NULL};
	return spawn_process(args);
	/* Something to do with socat */
}

static void shutdown_loopback(pid_t pid)
{
	kill(pid, SIGHUP);
	waitpid(pid, NULL, 0);
	/* kill off loopback */
}

void test_open(void)
{
	struct simple_uart *uart;
	pid_t pid;

	pid = setup_loopback();

	uart = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
	TEST_ASSERT(uart != NULL);
	simple_uart_close(uart);

	shutdown_loopback(pid);
}

void test_loopback(void)
{
	struct simple_uart *u1, *u2;
	char buffer[10];
	pid_t pid;

	pid = setup_loopback();

	u1 = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
	TEST_ASSERT(u1 != NULL);
	u2 = simple_uart_open(UART_LOOPBACK_2, 115200, "8N1");
	TEST_ASSERT(u2 != NULL);

	TEST_ASSERT(simple_uart_write(u1, "foo", 3) == 3);
	usleep(1000);
	TEST_ASSERT(simple_uart_read(u2, buffer, sizeof(buffer)) == 3);
	TEST_ASSERT(memcmp(buffer, "foo", 3) == 0);

	TEST_ASSERT(simple_uart_write(u2, "fnord", 5) == 5);
	usleep(1000);
	TEST_ASSERT(simple_uart_read(u1, buffer, sizeof(buffer)) == 5);
	TEST_ASSERT(memcmp(buffer, "fnord", 5) == 0);

	simple_uart_close(u1);
	simple_uart_close(u2);
	shutdown_loopback(pid);
}

TEST_LIST = {
   {"open", test_open},
   {"loopback", test_loopback},
   { NULL, NULL }
};