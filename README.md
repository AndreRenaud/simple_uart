# Simple UART

<img src="/simple_uart_logo.png" alt="Simple UART Logo" width="480"/>

[![GitHub Actions](https://github.com/AndreRenaud/simple_uart/workflows/C/C++%20CI/badge.svg)](https://github.com/AndreRenaud/simple_uart/actions)
[![License](https://img.shields.io/badge/License-BSD%200--Clause-brightgreen.svg)](/LICENSE)

Simple UART is a cross-platform serial port UART library. It supports Linux, Windows and macOS. It is designed to be simple to add to a project - it is a single .c and .h file with no dependencies beyond the standard system library.

## License

The code is licensed under the 0-Clause BSD - see [LICENSE](/LICENSE) for details.

## How to add it to a project

The library consists of a single .c and .h pair. To add it to a project, simply copy the *simple_uart.c* and *simple_uart.h* file into the project and add to the build system. There are no external dependencies beyond the standard system library, and the code should automatically determine which platform it is building for and comple correctly. The license should be suitable for both open-source and commercial projects.

Please submit a bug report if this does not work.

## API
```c
int simple_uart_list(char ***namesp, char ***descriptionp);
```

List available UART ports and descriptions.
It is the callers responsibility to free each element in namesp & descriptionsp after calling, as well as the top-level pointers themselves.

#### Arguments:
Argument | Details
-------- | -------
*namesp* | Pointer to a list of UART devices to be returned
*descriptionp* | Pointer to a list of UART description strings to be filled in (optional)

#### Return:
On success the number of elements in the *namesp* and *descriptionsp* lists will be returned. *< 0* on failure.

#### Example:
```c
char **names;
char **description;
int nuarts = simple_uart_list(&names, &descriptions);
for (int i = 0; i < nuarts; i++) {
	printf("Port %d: %s: %s\n", i, names[i], descriptions[i]);
}

for (int i = 0; i < nuarts; i++) {
	free(names[i]);
	free(descriptions[i]);
}
free(names);
free(descriptions);
```

```c
struct simple_uart *simple_uart_open(const char *name, int speed, const char *mode_string);
```
Open a given UART device.

#### Arguments:
Argument | Details
-------- | -------
*name*   | UART device name, such as "/dev/ttyUSB0", or "COM1:". These can be found via simple_uart_list
*speed*  | Baudrate to open the port at, ie: 115200
*mode_string*| Mode to open the port at, such as "8N1", or "7E2" - indicating number of data bits, parity bit and number of stop bits. This is most commonly "8N1". Also accepts 'F' to enable hardware flow control (RTS/CTS), and 'R' to enable RS485 mode.

#### Return:
On success, a new opaque simple_uart structure is returned. On failure, NULL is returned.

```c
int simple_uart_close(struct simple_uart *uart);
```
Close a previously opened UART.

```c
int simple_uart_read(struct simple_uart *uart, void *buffer, int max_len);
```
Read bytes from the given UART.

#### Arguments:
Argument | Details
-------- | -------
*uart*   | UART to read from
*buffer* | Area to store read data into
*max_len*| Maximum number of bytes to store into *buffer*

#### Return:
Returns the number of bytes written to *buffer* (up to *max_len*). < 0 on failure.

```c
int simple_uart_write(struct simple_uart *uart, const void *buffer, int len);
```
Write bytes to the given UART.

#### Arguments:
Argument | Details
-------- | -------
*uart*   | UART to write to
*buffer* | Buffer to write data from
*len*    | Number of bytes to write

#### Return:
Returns the number of bytes written from *buffer* to the UART. < 0 on failure.

```c
int simple_uart_set_character_delay(struct simple_uart *uart, int delay_us);
```
Sets a delay to be specified between each character written when using *simple_uart_write*.
This is useful when communicating with devices that do not support flow control, and are not able to process data at full baudrate

```c
bool simple_uart_has_data(struct simple_uart *uart);
```
Determine if there is outstanding data available to read

```c
int simple_uart_set_logfile(struct simple_uart *uart, const char *logfile, ...);
```
Store all read/written data into the given file. Filename can be specified using printf-style syntax.

```c
int simple_uart_get_fd(struct simple_uart *uart);
```
*POSIX ONLY* Return the file descriptor for the UART. Useful for monitoring for new data.

```c
HANDLE simple_uart_get_handle(struct simple_uart *uart);
```
*WIN32 ONLY* Return the Windows handle for the UART. Useful for monitoring for new data.

```c
int simple_uart_get_pin(struct simple_uart *uart, int pin);
```
Get the state of a hardware pin of the UART.

#### Arguments:
Argument | Details
-------- | -------
*uart*   | UART to set the pin on
*pin*    | One of: SIMPLE_UART_RTS, SIMPLE_UART_CTS, SIMPLE_UART_DSR, SIMPLE_UART_DCD, SIMPLE_UART_DTR, SIMPLE_UART_RI

```c
int simple_uart_set_pin(struct simple_uart *uart, int pin, bool high);
```
Set the state of a hardware pin of the UART. See *simple_uart_get_pin*

```c
int simple_uart_flush(struct simple_uart *uart);
```
Ensure all outstanding data has been written to the wire and is not being buffered by the OS.

```c
int simple_uart_send_break(struct simple_uart *uart);
```
Send a serial break on the UART

```c
int simple_uart_read_line(struct simple_uart *uart, char *result, int max_len, int ms_timeout);
```
Read bytes of data up to a carriage return/line feed.