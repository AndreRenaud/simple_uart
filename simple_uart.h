#ifndef SIMPLE_UART_H
#define SIMPLE_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

struct simple_uart;

enum {
	SIMPLE_UART_RTS,
	SIMPLE_UART_CTS,
	SIMPLE_UART_DSR,
	SIMPLE_UART_DCD,
	SIMPLE_UART_DTR,
	SIMPLE_UART_RI,
};

// Builds a list of device names that can be opened.
// ie: /dev/ttyS0 etc... (or COM1: on Win32)
// Returns the number of items in the list.
// Caller is responsible for free'ing each name/description and the overall list
int simple_uart_list(char ***namesp, char ***descriptionp);

/**
 * Opens a uart, either by device name, or if that fails it will search
 * for a substring match of the descriptions, as returned by simple_uart_list
 */
struct simple_uart *simple_uart_open(const char *name, int speed, const char *mode_string);
int simple_uart_close(struct simple_uart *uart);

/* Raw read/write */
int simple_uart_read(struct simple_uart *uart, void *buffer, int max_len);
int simple_uart_write(struct simple_uart *uart, const void *buffer, int len);

/* Sets the delay between sending each character when using uart_write */
int simple_uart_set_character_delay(struct simple_uart *uart, int delay_us);

/**
 * Returns how many data bytes available on the uart
 */
int simple_uart_has_data(struct simple_uart *uart);

/**
 * Set the logfile to store all read/write data to
 */
int simple_uart_set_logfile(struct simple_uart *uart, const char *logfile, ...);

int simple_uart_get_fd(struct simple_uart *uart);
#ifdef WIN32
HANDLE simple_uart_get_handle(struct simple_uart *uart);
#endif

/**
 *
 */
int simple_uart_get_pin(struct simple_uart *uart, int pin);

int simple_uart_set_pin(struct simple_uart *uart, int pin, bool high);

/**
 * Make sure all write data has been flushed out of the UART
 */
int simple_uart_flush(struct simple_uart *uart);

/**
 * Send the break symbol
 */
int simple_uart_send_break(struct simple_uart *uart);

/**
 * Read data until either a line ending (carriage return/line feed) is seen,
 * or a timeout occurs
 */
int simple_uart_read_line(struct simple_uart *uart, char *result, int max_len, int ms_timeout);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_UART_H */