#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>

#include "simple_uart.h"

static void print_usage(const char *program)
{
	fprintf(stderr, "%s [-p port] [-b baudrate] [-f flags] | -l\n", program);
}

int main(int argc, char *argv[])
{
	const char *port = NULL;
	int baudrate;
	const char *flags = "8N1";
	int opt;

	while ((opt = getopt(argc, argv, "p:b:f:lh")) != -1) {
		switch (opt) {
			case 'p': port = optarg; break;
			case 'b': baudrate = atoi(optarg); break;
			case 'f': flags = optarg; break;
			case 'l': {
				char **names;
				char **descriptions;
				int len = simple_uart_list(&names, &descriptions);
				printf("Found %d ports\n", len);
				for (int i = 0; i < len; i++) {
					printf("%2d) %-40s - %s\n", i, names[i], (descriptions && descriptions[i]) ? descriptions[i] : "unknown");
				}
				for (int i = 0; i < len; i++) {
					if (names[i]) free(names[i]);
					if (descriptions && descriptions[i]) free(descriptions[i]);
				}
				free(names);
				if (descriptions) free(descriptions);
				exit(0);
			}
			default:
			case 'h':
				print_usage(argv[0]);
				exit(1);
		}

	}

	if (!port) {
		print_usage(argv[0]);
		exit(1);
	}

	struct simple_uart *uart;

	uart = simple_uart_open(port, baudrate, flags);
	if (!uart) {
		fprintf(stderr, "Unable to open %s:%d:%s\n", port, baudrate, flags);
		exit(1);
	}

    /* Disable buffering on stdin */
	struct termios term;

	if (tcgetattr(STDIN_FILENO, &term)) {
    	fprintf(stderr, "tcgetattr\n");
    	exit(-1);
  	}

  	term.c_lflag &= ~(ICANON | ECHO);
  	term.c_cc[VMIN] = 0;
  	term.c_cc[VTIME] = 0;

  	if (tcsetattr(STDIN_FILENO, TCSANOW, &term)) {
    	fprintf(stderr, "tcsetattr\n");
    	exit(-1);
  	}

	while (1) {
		fd_set fds;
		int nfds = STDIN_FILENO;
		struct timeval tv = {1, 0};
		char buffer[128];

		/* Wait for bytes on either serial port or stdin */
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		FD_SET(simple_uart_get_fd(uart), &fds);
		if (simple_uart_get_fd(uart) > nfds)
			nfds = simple_uart_get_fd(uart);

		select(nfds + 1, &fds, NULL, NULL, &tv);

		/* Move bytes from one to the other */
		if (FD_ISSET(STDIN_FILENO, &fds)) {
			int len = read(STDIN_FILENO, buffer, sizeof(buffer));
			if (len > 0) simple_uart_write(uart, buffer, len);
		}
		if (FD_ISSET(simple_uart_get_fd(uart), &fds)) {
			int len = simple_uart_read(uart, buffer, sizeof(buffer));
			if (len > 0) write(STDOUT_FILENO, buffer, len);
		}
	}

	simple_uart_close(uart);
	return 0;
}