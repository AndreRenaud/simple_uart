# Simple UART

<img src="/simple_uart_logo.png" alt="Simple UART Logo" width="480"/>

[![GitHub Actions](https://github.com/AndreRenaud/simple_uart/workflows/C/C++%20CI/badge.svg)](https://github.com/AndreRenaud/simple_uart/actions)
[![License](https://img.shields.io/badge/License-BSD%200--Clause-brightgreen.svg)](/LICENSE)

Simple UART is a cross-platform serial port UART library. It supports Linux, Windows and macOS. It is designed to be simple to add to a project - it is a single .c and .h file with no dependencies beyond the standard system library.

## License

The code is licensed under the 0-Clause BSD - see [LICENSE](/LICENSE) for details.

## API
```c
int simple_uart_list(char ***namesp, char ***descriptionp);
```

List available UART ports and descriptions.
It is the callers responsibility to free each element in namesp & descriptionsp after calling, as well as the top-level pointers themselves.

#### Arguments:
Arg | Description
--- | -----------
*namesp* | Pointer to a list of UART devices to be returned
*descriptionp* | Pointer to a list of UART description strings to be filled in (optional)

#### Return:
On success the number of elements in the *namesp* and *descriptionsp* lists will be returned. *< 0* on failure.

### Example:
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
```c


```c
struct simple_uart *simple_uart_open(const char *name, int speed, const char *mode_string);
```

```c
int simple_uart_close(struct simple_uart *uart);
```

```c
int simple_uart_read(struct simple_uart *uart, void *buffer, int max_len);
```

```c
int simple_uart_write(struct simple_uart *uart, const void *buffer, int len);
```

```c
int simple_uart_set_character_delay(struct simple_uart *uart, int delay_us);
```

```c
bool simple_uart_has_data(struct simple_uart *uart);
```

```c
int simple_uart_set_logfile(struct simple_uart *uart, const char *logfile, ...);
```

```c
int simple_uart_get_fd(struct simple_uart *uart);
```

```c
HANDLE simple_uart_get_handle(struct simple_uart *uart);
```

```c
int simple_uart_get_pin(struct simple_uart *uart, int pin);
```

```c
int simple_uart_set_pin(struct simple_uart *uart, int pin, bool high);
```

```c
int simple_uart_flush(struct simple_uart *uart);
```

```c
int simple_uart_send_break(struct simple_uart *uart);
```

```c
int simple_uart_read_line(struct simple_uart *uart, char *result, int max_len, int ms_timeout);
```
