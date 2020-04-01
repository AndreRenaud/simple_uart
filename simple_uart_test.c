#include "acutest.h"
#include "simple_uart.h"

#define UART_LOOPBACK_1 "/dev/loopback"
#define UART_LOOPBACK_2 "/dev/loopback"

static void setup_loopback(void)
{
	/* Something to do with losetup */
}

void test_open(void)
{
	struct simple_uart *uart;

	setup_loopback();

	uart = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
	TEST_ASSERT(uart != NULL);
	simple_uart_close(uart);
}

TEST_LIST = {
   { "open", test_open },
   { NULL, NULL }
};