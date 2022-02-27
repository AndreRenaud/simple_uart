#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <regex.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#ifndef WIN32
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <glob.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <libgen.h>
#else
#include <windows.h>
#endif

#ifdef __linux__
#include <pty.h>
#include <linux/serial.h>
#endif

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

#include "simple_uart.h"

#if defined(__linux__) || defined(__APPLE__)
/* This requires you to link against -lrt
 */
#define mseconds() (int)({ struct timespec _ts; \
                      clock_gettime(CLOCK_MONOTONIC, &_ts); \
                      _ts.tv_sec * 1000 + _ts.tv_nsec / (1000 * 1000); \
                   })
#else
#define mseconds() GetTickCount()
#endif

#ifndef TIOCSRS485
#define TIOCSRS485   0x542F
#endif
#ifndef SER_RS485_RX_DURING_TX
#define SER_RS485_RX_DURING_TX  (1 << 4)
#endif

struct simple_uart
{
#ifdef WIN32
    HANDLE port;
#else
    int fd;
#endif
    int char_delay_us;
    FILE *logfile;
};

int simple_uart_read(struct simple_uart *sc, void *buffer, int max_len)
{
#ifdef WIN32
    COMMTIMEOUTS commTimeout;

    /* Get the comm timouts */
    if (GetCommTimeouts(sc->port, &commTimeout)) {
        /* Set the timeout to return immediately with whatever data is there */
        commTimeout.ReadIntervalTimeout = MAXDWORD;
        commTimeout.ReadTotalTimeoutMultiplier = MAXDWORD;
        commTimeout.ReadTotalTimeoutConstant = 1;
        SetCommTimeouts(sc->port, &commTimeout);
    }

    if (!ReadFile (sc->port, buffer, max_len, (LPDWORD)&r, NULL) != 0)
        return -GetLastError();
#else
    fd_set readfds, exceptfds;
    struct timeval t;
    int r;

    // Have a timeout of 50ms to avoid just thrashing the CPU
    FD_ZERO(&readfds);
    FD_SET(sc->fd, &readfds);
    FD_ZERO(&exceptfds);
    FD_SET(sc->fd, &exceptfds);
    t.tv_sec = 0;
    t.tv_usec = 50 * 1000;

    r = select(sc->fd + 1, &readfds, NULL, &exceptfds, &t);
    if (r < 0)
        return -errno;
    if (r == 0)
        return 0;
    r = read (sc->fd, buffer, max_len);
    if (r < 0)
        r = -errno;
#endif
    if (r > 0 && sc->logfile) {
        fwrite(buffer, r, 1, sc->logfile);
        fflush(sc->logfile);
    }
    return r;
}

int simple_uart_write(struct simple_uart *sc, const void *buffer, int len)
{
#ifdef WIN32
    int r = 0;
    /* TODO: Support char_delay_us */
    WriteFile (sc->port, buffer, len, (LPDWORD)&r, NULL);
#else
    int r;
    if (sc->char_delay_us > 0) {
        const uint8_t *buf8 = buffer;
        for (int i = 0; i < len; i++) {
            int e = write(sc->fd, &buf8[i], 1);
            if (e < 0)
                return e;
            if (e == 0)
                return i;
            usleep(sc->char_delay_us);
        }
        r = len;
    } else {
        r = write (sc->fd, buffer, len);
        if (r < 0)
            r = -errno;
    }
#endif

    if (r > 0 && sc->logfile) {
        fwrite(buffer, r, 1, sc->logfile);
        fflush(sc->logfile);
    }

    return r;
}

static int simple_uart_set_config(struct simple_uart *sc, int speed, const char *mode_string)
{
#ifdef WIN32
    DCB dcbConfig;
    if (GetCommState (sc->port, &dcbConfig)) {
        // TODO: Do mode_string properly
        dcbConfig.BaudRate = speed;
        dcbConfig.ByteSize = 8;
        dcbConfig.Parity = NOPARITY;
        dcbConfig.StopBits = ONESTOPBIT;
        dcbConfig.fBinary = TRUE;
        dcbConfig.fParity = TRUE;
        SetCommState (sc->port, &dcbConfig);
    }
    return 0;
#else
    struct termios options;
    int sp = 0;
#ifdef __linux__
    int non_standard = 0;
#else // __APPLE__ type casting
    speed_t mac_speed = speed;
#endif

    switch (speed)
    {
    case 1200:
        sp = B1200;
        break;
    case 2400:
        sp = B2400;
        break;
    case 4800:
        sp = B4800;
        break;
    case 9600:
        sp = B9600;
        break;
    case 19200:
        sp = B19200;
        break;
    case 38400:
        sp = B38400;
        break;
    case 57600:
        sp = B57600;
        break;
    case 115200:
        sp = B115200;
        break;
    case 230400:
        sp = B230400;
        break;
#ifdef __linux__
    case 460800:
        sp = B460800;
        break;
    case 500000:
        sp = B500000;
        break;
    case 576000:
        sp = B576000;
        break;
    case 921600:
        sp = B921600;
        break;
    case 1000000:
        sp = B1000000;
        break;
    case 1152000:
        sp = B1152000;
        break;
    case 1500000:
        sp = B1500000;
        break;
    case 2000000:
        sp = B2000000;
        break;
    case 2500000:
        sp = B2500000;
        break;
    case 3000000:
        sp = B3000000;
        break;
    case 3500000:
        sp = B3500000;
        break;
    case 4000000:
        sp = B4000000;
        break;

    default:
        sp = B38400;
        non_standard = 1;
#else // __APPLE__ only
    // The IOSSIOSPEED ioctl can be used to set arbitrary baud rates
    // other than those specified by POSIX. The driver for the underlying serial hardware
    // ultimately determines which baud rates can be used. This ioctl sets both the input
    // and output speed.
    default: {
        int r = ioctl(sc->fd, IOSSIOSPEED, &mac_speed);
        if (r < 0 && errno != ENOTTY)
            return -errno;
    }
#endif
    }

    if (tcgetattr(sc->fd, &options) < 0)
        return -errno;

    if (sp != 0) {
        cfsetospeed(&options, sp);
        cfsetispeed(&options, sp);
    }

    options.c_cflag &= ~(HUPCL);

    options.c_cflag |= CREAD | CLOCAL;

#define HAS_OPTION(a) (strchr (mode_string, a) != NULL || strchr (mode_string, tolower(a)) != NULL)

    // parity
    if (HAS_OPTION ('N'))
        options.c_cflag &= ~PARENB;
    else if (HAS_OPTION ('E')) {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
    } else if (HAS_OPTION ('O')) {
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
    }
    // stop bits
    if (HAS_OPTION ('2'))
        options.c_cflag |= CSTOPB;
    else
        options.c_cflag &= ~CSTOPB;
    /* Flush data on each write */
    if (HAS_OPTION('W'))
        options.c_lflag |= NOFLSH;

    // Character size
    options.c_cflag &= ~CSIZE;
    if (HAS_OPTION ('8'))
        options.c_cflag |= CS8;
    else if (HAS_OPTION ('7'))
        options.c_cflag |= CS7;
    else if (HAS_OPTION ('6'))
        options.c_cflag |= CS6;
    else if (HAS_OPTION ('5'))
        options.c_cflag |= CS5;

    /* Flow control */
    if (HAS_OPTION('F'))
        options.c_cflag |= CRTSCTS;
    else
        options.c_cflag &= ~CRTSCTS;

    // raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // disable software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // maintain carriage return on input, and don't translate it
    options.c_iflag &= ~(IGNCR | ICRNL | INLCR);

    // raw output mode
    options.c_oflag &= ~OPOST;

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(sc->fd, TCIOFLUSH);
    if (tcsetattr(sc->fd, TCSANOW, &options) < 0)
        return -errno;

#ifdef __linux__
    if (HAS_OPTION('R')) {
        struct serial_rs485 rs485;
        rs485.flags = SER_RS485_ENABLED | SER_RS485_RX_DURING_TX | SER_RS485_RTS_ON_SEND;
        if (ioctl(sc->fd, TIOCSRS485, &rs485) < 0)
            return -errno;
    }

    if (non_standard) {
        struct serial_struct ss;

        /* Get current settings */
        if (ioctl(sc->fd, TIOCGSERIAL, &ss) < 0)
            return -errno;
        /* Check we can divide down */
        if ((ss.baud_base / speed) == 0)
            return -EINVAL;
        ss.flags &= ~(ASYNC_SPD_MASK);
        ss.flags |= ASYNC_SPD_CUST;
        ss.custom_divisor = ss.baud_base / speed;
        if (ioctl(sc->fd, TIOCSSERIAL, &ss) < 0)
            return -errno;
    }
#endif
    return 0; 
#endif
}

int simple_uart_close(struct simple_uart *sc)
{
    if (!sc)
        return -EINVAL;

#if defined(__linux__) || defined(__APPLE__)
    close(sc->fd);
#else
    CloseHandle(sc->port);
#endif
    free(sc);

    return 0;
}

struct simple_uart *simple_uart_open(const char *device, int speed, const char *mode_string)
{
    struct simple_uart *retval;

#ifdef WIN32
    HANDLE port;
    DWORD mode = GENERIC_READ | GENERIC_WRITE;
    char full_port_name[32];

    snprintf(full_port_name, sizeof(full_port_name), "\\\\.\\%s", device);
    full_port_name[sizeof(full_port_name) - 1] = '\0';

    port = CreateFile (full_port_name, mode, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (port == INVALID_HANDLE_VALUE)
        return NULL;
    retval = calloc(sizeof(struct simple_uart), 1);
    retval->port = port;
#else
    int fd;
    int mode = O_RDWR | O_NDELAY | O_NOCTTY;

    fd = open(device, mode);

    if (fd == -1)
        return NULL;

    signal(SIGIO, SIG_IGN); // so we don't get those 'I/O possible' lines

    fcntl(fd, F_SETFL, 0);
    retval = calloc(sizeof(struct simple_uart), 1);
    if (!retval)
    {
        close(fd);
        return NULL;
    }
    retval->fd = fd;
#endif
    int r = simple_uart_set_config(retval, speed, mode_string);
    if (r < 0 && r != -ENOTTY) {
        simple_uart_close(retval);
        return NULL;
    }
    return retval;
}

int simple_uart_has_data(struct simple_uart *sc)
{
    if (!sc)
        return 0;

#ifdef WIN32
    COMSTAT cs;
    if (!ClearCommError(sc->port, NULL, &cs)) {
        return 0;
    }
    return cs.cbInQue;
#else
    int bytes_available;
    if (ioctl(sc->fd, FIONREAD, &bytes_available) < 0)
        return -errno;
    return bytes_available;
#endif
}

int simple_uart_set_character_delay(struct simple_uart *sc, int delay_us)
{
    int old_delay = sc->char_delay_us;
    sc->char_delay_us = delay_us;
    return old_delay;
}

int simple_uart_list(char ***namesp, char ***descriptionp)
{
#if defined(__linux__) || defined(__APPLE__)
    glob_t g;
    char **names = NULL;
    char **description = NULL;
    int count = 0;

#ifdef __linux__
    if (glob("/sys/class/tty/ttyS[0-9]*", 0, NULL, &g) >= 0) {
        char buffer[100];
        char **new_names;
        new_names = realloc(names, (count + g.gl_pathc) * sizeof(char *));
        if (!new_names) {
            globfree(&g);
            free(names);
            return -ENOMEM;
        }
        names = new_names;
        for (int i = count; i < count + g.gl_pathc; i++) {
            sprintf(buffer, "/dev/%s", basename(g.gl_pathv[i - count]));
            names[i] = strdup(buffer);
        }
        count += g.gl_pathc;
        globfree (&g);
    }

    if (glob ("/sys/class/tty/ttyUSB[0-9]*", 0, NULL, &g) >= 0) {
        char buffer[100];
        char **new_names;
        new_names = realloc(names, (count + g.gl_pathc) * sizeof (char *));
        if (!new_names) {
            globfree(&g);
            free(names);
            return -ENOMEM;
        }
        names = new_names;
        for (int i = count; i < count + g.gl_pathc; i++) {
            sprintf (buffer, "/dev/%s", basename (g.gl_pathv[i - count]));
            names[i] = strdup (buffer);
        }
        count += g.gl_pathc;
        globfree (&g);
    }
#endif

#ifdef __APPLE__
        if (glob ("/dev/tty.*", 0, NULL, &g) >= 0) {
        char buffer[100];
        char **new_names;
        new_names = realloc(names, (count + g.gl_pathc) * sizeof (char *));
        if (!new_names) {
            globfree(&g);
            free(names);
            return -ENOMEM;
        }
        names = new_names;
        for (int i = count; i < count + g.gl_pathc; i++) {
            sprintf (buffer, "/dev/%s", basename (g.gl_pathv[i - count]));
            names[i] = strdup (buffer);
        }
        count += g.gl_pathc;
        globfree (&g);
    }
#endif

    *namesp = names;
    *descriptionp = description;
    return count;
#else
    int pos = 0;
    char **names = NULL;

    for (int i = 0; i < 255; i++) {
        char buffer[10];
        char target[255];
        sprintf(buffer, "COM%d", i + 1);
        if (QueryDosDevice(buffer, target, sizeof(target)) > 0) {
            char **new_names = realloc(names, (pos + 1) * sizeof (char *));
            if (!new_names)
                continue;
            names = new_names;
            names[pos] = malloc(strlen (buffer) + 1);
            strcpy(names[pos], buffer);
            pos++;
        }
    }
    *namesp = names;
    return pos;
#endif
}

int simple_uart_set_logfile(struct simple_uart *uart, const char *logfile, ...)
{
    va_list ap;
    char *buffer;
    int len;

    va_start(ap, logfile);
    len = vasprintf(&buffer, logfile, ap);
    va_end(ap);
    if (len < 0)
        return -errno;
    if (uart->logfile) {
        fclose(uart->logfile);
        uart->logfile = NULL;
    }
    uart->logfile = fopen(buffer, "a");
    if (!uart->logfile) {
        int e = -errno;
        free(buffer);
        return e;
    }
    free(buffer);
    return 0;
}

int simple_uart_read_line(struct simple_uart *uart, char *buffer, int max_len, int timeout)
{
    int pos = 0;
    int last = mseconds();
    int now;

    if (!buffer || max_len == 0)
        return -EINVAL;

    *buffer = '\0';

    do {
        char ch;
        int ret = simple_uart_read(uart, &ch, 1);
        if (ret < 0)
            return ret;
        if (ret > 0) {
            if (ch == '\n' || ch == '\r') {
                break;
            } else {
                last = mseconds();
                buffer[pos] = ch;
                buffer[pos + 1] = '\0';
                pos++;
            }
        }
        now = mseconds();
    } while (pos < max_len - 1 && now - last < timeout);

    /* If we got a carriage return + line feed, just collapse them */
    while (pos > 0 && (buffer[pos - 1] == '\r' || buffer[pos - 1] == '\n')) {
        buffer[pos - 1] = '\0';
        pos--;
    }

    if (now - last >= timeout)
        return -ETIMEDOUT;

    return pos;
}

int simple_uart_send_break(struct simple_uart *uart)
{
#if defined(__linux__) || defined(__APPLE__)
    tcsendbreak(uart->fd, 1);
    return 0;
#else
    SetCommBreak(uart->port);
    // Linux doesn't support durations, it is always 4/10 of a second.
    // Replicate that here.
    msleep(400);
    ClearCommBreak(uart->port);
    return 0;
#endif
}

#ifdef WIN32
HANDLE simple_uart_get_handle(struct simple_uart *uart)
{
    return uart->port;
}
#else
int simple_uart_get_fd(struct simple_uart *uart)
{
    return uart->fd;
}
#endif

int simple_uart_get_pin(struct simple_uart *uart, int pin)
{
#if defined(__linux__) || defined(__APPLE__)
    int status;

    if (ioctl(uart->fd, TIOCMGET, &status) < 0)
        return -errno;

    switch (pin) {
    case SIMPLE_UART_CTS:
        return (status & TIOCM_CTS) ? 1 : 0;
    case SIMPLE_UART_DSR:
        return (status & TIOCM_DSR) ? 1 : 0;
    case SIMPLE_UART_DCD:
        return (status & TIOCM_CAR) ? 1 : 0;
    case SIMPLE_UART_RI:
        return (status & TIOCM_RI) ? 1 : 0;
    default:
        return -EINVAL;
    }
#else
    DWORD status;
    if (!GetCommModemStatus(uart->port, &status))
        return -GetLastError();

    switch (pin) {
    case SIMPLE_UART_CTS:
        return (status & MS_CTS_ON) ? 1 : 0;
    case SIMPLE_UART_DSR:
        return (status & MS_DSR_ON) ? 1 : 0;
    case SIMPLE_UART_DCD:
        return (status & MS_RSLD_ON) ? 1 : 0;
    case SIMPLE_UART_RI:
        return (status & MS_RING_ON) ? 1 : 0;
    default:
        return -EINVAL;
    }
#endif
}

int simple_uart_set_pin(struct simple_uart *uart, int pin, bool high)
{
#if defined(__linux__) || defined(__APPLE__)
    int bits;

    if (!uart || uart->fd < 0)
        return -EINVAL;

    switch (pin) {
    case SIMPLE_UART_RTS:
        bits = TIOCM_RTS;
        break;
    case SIMPLE_UART_DTR:
        bits = TIOCM_DTR;
        break;
    default:
        return -EINVAL;
    }
    if (ioctl(uart->fd, high ? TIOCMBIS : TIOCMBIC, &bits) < 0)
        return -errno;

    return 0;
#else
    bool res;
    switch(pin) {
    case SIMPLE_UART_RTS:
        res = EscapeCommFunction(uart->port, high ? SETRTS : CLRRTS);
        break;
    case SIMPLE_UART_DTR:
        res = EscapeCommFunction(uart->port, high ? SETDTR : CLRDTR);
        break;
    default:
        return -EINVAL;
    }
    if (!res)
        return -GetLastError();
    return 0;
#endif
}
