CFLAGS=-g -Wall -pipe

default: simple_uart_test simple_uart_term

test: simple_uart_test
	./simple_uart_test

check:
	cppcheck -q --enable=all -UTEST_FINI -UTEST_INIT -UCLOCK_MONOTONIC -URUNNING_ON_VALGRIND *.c

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

simple_uart_test: simple_uart_test.o simple_uart.o
	$(CC) -o simple_uart_test simple_uart_test.o simple_uart.o

simple_uart_term: simple_uart_term.o simple_uart.o
	$(CC) -o simple_uart_term simple_uart_term.o simple_uart.o

clean:
	rm -f simple_uart_test *.o

.PHONY: clean check test