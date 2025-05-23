SRCS=simple_uart.c simple_uart.h simple_uart_test.c simple_uart_term.c
CFLAGS=-g -Wall -pipe

default: simple_uart_test simple_uart_term

test: simple_uart_test
	./simple_uart_test --xml-output=test-results.xml

format:
	clang-format -i $(SRCS)

check:
	cppcheck -q --enable=all -UTEST_FINI -UTEST_INIT -UCLOCK_MONOTONIC -URUNNING_ON_VALGRIND *.c
	clang-format --dry-run $(SRCS)

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

simple_uart_test: simple_uart_test.o simple_uart.o
	$(CC) -o simple_uart_test simple_uart_test.o simple_uart.o

simple_uart_term: simple_uart_term.o simple_uart.o
	$(CC) -o simple_uart_term simple_uart_term.o simple_uart.o

clean:
	rm -f simple_uart_test *.o test-results.xml

.PHONY: clean check test format
