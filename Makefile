SRCS=simple_uart.c simple_uart.h simple_uart_test.c simple_uart_term.c
CFLAGS=-g -Wall -pipe
CLANG_FORMAT=clang-format

default: simple_uart_test simple_uart_term

test: simple_uart_test
	./simple_uart_test --xml-output=test-results.xml

valgrind: simple_uart_test
	valgrind --leak-check=full --show-leak-kinds=all ./simple_uart_test

format:
	$(CLANG_FORMAT) -i $(SRCS)

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

.PHONY: clean check test format valgrind
