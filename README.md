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

## [API](./simple_uart.h)

### List
```c
ssize_t simple_uart_list(char ***namesp);
```

List available UART ports.
It is the callers responsibility to free each element in *namesp* after calling, as well as the top-level pointers themselves.

#### Arguments:
Argument | Details
-------- | -------
*namesp* | Pointer to a list of UART devices to be returned

#### Return:
On success the number of elements in the *namesp* list will be returned. *< 0* on failure.

#### Example:
```c
char **names;
ssize_t nuarts = simple_uart_list(&names);
for (ssize_t i = 0; i < nuarts; i++) {
    printf("Port %d: %s\n", i, names[i]);
}

for (ssize_t i = 0; i < nuarts; i++) {
    if (names[i]) free(names[i]);
}
free(names);
```


### Describe
```c
int simple_uart_describe(const char *uart, char *description, size_t max_desc_len);
```

Get UART device hardware description:
* Linux: ```manufacturer='<>',product='<>',PID=<>,VID=<>,serial=<>```
* Windows: ```PID=<>,VID=<>```

#### Arguments:
Argument | Details
-------- | -------
*uart*   | UART device to examine
*description* | String with port description
*max_desc_len*| Maximum number of bytes in *description*

#### Return:
On success *== 0*. *!= 0* on failure.


### Open
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


### Read
```c
ssize_t simple_uart_read(struct simple_uart *uart, void *buffer, size_t max_len);
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


### Write
```c
ssize_t simple_uart_write(struct simple_uart *uart, const void *buffer, size_t len);
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


### Delay
```c
unsigned int simple_uart_set_character_delay(struct simple_uart *uart, unsigned int delay_us);
```
Sets a delay to be specified between each character written when using *simple_uart_write*.
This is useful when communicating with devices that do not support flow control, and are not able to process data at full baudrate


### Data available
```c
int simple_uart_has_data(struct simple_uart *uart);
```
Returns the number of available UART bytes.


### Logfile
```c
int simple_uart_set_logfile(struct simple_uart *uart, const char *logfile, ...);
```
Store all read/written data into the given file. Filename can be specified using printf-style syntax.


### Get file descriptor
```c
int simple_uart_get_fd(struct simple_uart *uart);
```
*POSIX ONLY* Return the file descriptor for the UART. Useful for monitoring for new data.


### Get handle
```c
HANDLE simple_uart_get_handle(struct simple_uart *uart);
```
*WIN32 ONLY* Return the Windows handle for the UART. Useful for monitoring for new data.


### Get pin
```c
int simple_uart_get_pin(struct simple_uart *uart, int pin);
```
Get the state of a hardware pin of the UART.

#### Arguments:
Argument | Details
-------- | -------
*uart*   | UART to set the pin on
*pin*    | One of: SIMPLE_UART_RTS, SIMPLE_UART_CTS, SIMPLE_UART_DSR, SIMPLE_UART_DCD, SIMPLE_UART_DTR, SIMPLE_UART_RI


### Set Pin
```c
int simple_uart_set_pin(struct simple_uart *uart, int pin, bool high);
```
Set the state of a hardware pin of the UART. See *simple_uart_get_pin*


### Flush
```c
int simple_uart_flush(struct simple_uart *uart);
```
Ensure all outstanding data has been written to the wire and is not being buffered by the OS.


### Break
```c
int simple_uart_send_break(struct simple_uart *uart);
```
Send a serial break on the UART


### Read line
```c
ssize_t simple_uart_read_line(struct simple_uart *uart, char *result, int max_len, int ms_timeout);
```
Read bytes of data up to a carriage return/line feed.
