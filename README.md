# Simple UART

Build: [![GitHub Actions](https://github.com/AndreRenaud/simple_uart/workflows/ccpp/badge.svg)](https://github.com/AndreRenaud/simple_uart/actions)
<img src="https://raw.githubusercontent.com/AndreRenaud/simple_uart/master/simple_uart_logo.png" alt="Simple UART Logo" width="630"/>

## API
```c
int simple_uart_list(char ***namesp, char ***descriptionp);
```

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
