#include "acutest.h"
#include "simple_uart.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef ETIMEDOUT
#define ETIMEDOUT 110
#endif

/* Timer function to get milliseconds - matches the one in simple_uart.c */
#if defined(__linux__) || defined(__APPLE__)
static uint64_t mseconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / (1000 * 1000);
}
#else
static uint64_t mseconds(void)
{
    return (uint64_t)GetTickCount64();
}
#endif

#define UART_LOOPBACK_1 "/tmp/ttyS0"
#define UART_LOOPBACK_2 "/tmp/ttyS1"

static pid_t setup_loopback(void)
{
    pid_t pid;
    char *const args[] = {"socat", "PTY,link=" UART_LOOPBACK_1 ",raw,echo=0,iexten=0",
                          "PTY,link=" UART_LOOPBACK_2 ",raw,echo=0,iexten=0", NULL};
    pid = fork();

    TEST_ASSERT(pid >= 0);

    if (pid == 0) {
        execvp(args[0], args);
    }
    /* Give it time to start */
    usleep(100000);
    return pid;
}

static void shutdown_loopback(pid_t pid)
{
    kill(pid, SIGHUP);
    waitpid(pid, NULL, 0);
    /* kill off loopback */
}

void test_open(void)
{
    struct simple_uart *uart;
    pid_t pid;

    pid = setup_loopback();

    uart = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
    TEST_ASSERT(uart != NULL);
    simple_uart_close(uart);

    shutdown_loopback(pid);
}

void test_loopback(void)
{
    struct simple_uart *u1, *u2;
    char buffer[10];
    pid_t pid;

    pid = setup_loopback();

    u1 = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
    TEST_ASSERT(u1 != NULL);
    u2 = simple_uart_open(UART_LOOPBACK_2, 115200, "8N1");
    TEST_ASSERT(u2 != NULL);

    TEST_ASSERT(simple_uart_write(u1, "foo", 3) == 3);
    usleep(1000);
    TEST_ASSERT(simple_uart_read(u2, buffer, sizeof(buffer)) == 3);
    TEST_ASSERT(memcmp(buffer, "foo", 3) == 0);

    TEST_ASSERT(simple_uart_write(u2, "fnord", 5) == 5);
    usleep(1000);
    TEST_ASSERT(simple_uart_read(u1, buffer, sizeof(buffer)) == 5);
    TEST_ASSERT(memcmp(buffer, "fnord", 5) == 0);

    simple_uart_close(u1);
    simple_uart_close(u2);
    shutdown_loopback(pid);
}

void test_loopback_random(void)
{
    struct simple_uart *u1, *u2;
    pid_t pid;

    pid = setup_loopback();

    u1 = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
    TEST_ASSERT(u1 != NULL);
    u2 = simple_uart_open(UART_LOOPBACK_2, 115200, "8N1");
    TEST_ASSERT(u2 != NULL);

    for (int i = 0; i < 100; i++) {
        unsigned char tx[100];
        unsigned char rx[sizeof(tx) * 2];
        int tx_len = 1 + (rand() % (sizeof(tx) - 1));
        for (int j = 0; j < tx_len; j++)
            tx[j] = rand() % 256;
        memset(rx, 0, sizeof(rx));
        TEST_ASSERT(simple_uart_write(u1, tx, tx_len) == tx_len);
        usleep(1000); // Give it time to get to the other end
        TEST_ASSERT(simple_uart_has_data(u2) == tx_len);
        TEST_ASSERT(simple_uart_read(u2, rx, sizeof(rx)) == tx_len);
        TEST_ASSERT(memcmp(rx, tx, tx_len) == 0);
    }

    simple_uart_close(u1);
    simple_uart_close(u2);
    shutdown_loopback(pid);
}

void test_read_line(void)
{
    struct simple_uart *u1, *u2;
    char buffer[10];
    pid_t pid;

    pid = setup_loopback();

    u1 = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
    TEST_ASSERT(u1 != NULL);
    u2 = simple_uart_open(UART_LOOPBACK_2, 115200, "8N1");
    TEST_ASSERT(u2 != NULL);

    simple_uart_write(u1, "a", 1);
    simple_uart_write(u1, "b", 1);
    simple_uart_write(u1, "\r", 1);
    TEST_ASSERT(simple_uart_read_line(u2, buffer, sizeof(buffer), 100) == 2);
    TEST_ASSERT(strcmp(buffer, "ab") == 0);

    simple_uart_write(u2, "cd\n", 3);
    TEST_ASSERT(simple_uart_read_line(u1, buffer, sizeof(buffer), 100) == 2);
    TEST_ASSERT(strcmp(buffer, "cd") == 0);

    simple_uart_write(u2, "ef", 2);
    TEST_ASSERT(simple_uart_read_line(u1, buffer, sizeof(buffer), 100) < 0);
    TEST_ASSERT(strcmp(buffer, "ef") == 0);

    simple_uart_close(u1);
    simple_uart_close(u2);
    shutdown_loopback(pid);
}

void test_read_timeout(void)
{
    struct simple_uart *u1, *u2;
    char buffer[10];
    pid_t pid;
    uint64_t start_time, end_time;

    pid = setup_loopback();

    u1 = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
    TEST_ASSERT(u1 != NULL);
    u2 = simple_uart_open(UART_LOOPBACK_2, 115200, "8N1");
    TEST_ASSERT(u2 != NULL);

    /* Test timeout=0 (non-blocking) - should return immediately */
    start_time = mseconds();
    ssize_t result = simple_uart_read_timeout(u1, buffer, sizeof(buffer), 0);
    end_time = mseconds();
    TEST_ASSERT(result == -ETIMEDOUT);          // Nothing was available, so it timed out
    TEST_ASSERT((end_time - start_time) < 100); // Should return quickly

    /* Try reading nothing */
    result = simple_uart_read_timeout(u1, buffer, 0, 0);
    TEST_ASSERT(result == 0); // Should return 0 for reading nothing

    /* Test with actual timeout - should timeout after specified period */
    start_time = mseconds();
    result = simple_uart_read_timeout(u1, buffer, sizeof(buffer), 200); // 200ms timeout
    end_time = mseconds();
    TEST_ASSERT(result == -ETIMEDOUT);           // Should timeout
    TEST_ASSERT((end_time - start_time) >= 180); // Should wait at least ~180ms
    TEST_ASSERT_((end_time - start_time) < 300, "!end (%llu) - start (%llu) < 300 (%llu)", end_time, start_time, end_time-start_time);  // But not much longer than 200ms

    /* Test successful read within timeout */
    simple_uart_write(u2, "test", 4);
    usleep(10000); // Small delay to ensure data is available
    start_time = mseconds();
    result = simple_uart_read_timeout(u1, buffer, sizeof(buffer), 1000); // 1 second timeout
    end_time = mseconds();
    TEST_ASSERT(result == 4); // Should read 4 bytes
    TEST_ASSERT(memcmp(buffer, "test", 4) == 0);
    TEST_ASSERT((end_time - start_time) < 100); // Should return quickly when data is available

    simple_uart_close(u1);
    simple_uart_close(u2);
    shutdown_loopback(pid);
}

void test_logfile(void)
{
    struct simple_uart *u1, *u2;
    char buffer[20];
    pid_t pid;
    const char *logfile = "/tmp/simple_uart_test.log";
    FILE *fp;
    char log_contents[100];

    /* Remove any existing log file */
    unlink(logfile);

    pid = setup_loopback();

    u1 = simple_uart_open(UART_LOOPBACK_1, 115200, "8N1");
    TEST_ASSERT(u1 != NULL);
    u2 = simple_uart_open(UART_LOOPBACK_2, 115200, "8N1");
    TEST_ASSERT(u2 != NULL);

    /* Set logfile on u1 */
    TEST_ASSERT(simple_uart_set_logfile(u1, logfile) == 0);

    /* Test error cases */
    TEST_ASSERT(simple_uart_set_logfile(NULL, logfile) == -EINVAL);
    TEST_ASSERT(simple_uart_set_logfile(u1, NULL) == -EINVAL);

    TEST_ASSERT(simple_uart_write(u1, "hello", 5) == 5);
    TEST_ASSERT(simple_uart_read(u2, buffer, sizeof(buffer)) == 5);
    TEST_ASSERT(simple_uart_write(u2, "world", 5) == 5);
    TEST_ASSERT(simple_uart_read(u1, buffer, sizeof(buffer)) == 5);

    /* Close connections to flush logs */
    simple_uart_close(u1);
    simple_uart_close(u2);
    shutdown_loopback(pid);

    /* Check that logfile was created and contains expected data */
    fp = fopen(logfile, "r");
    TEST_ASSERT(fp != NULL);

    size_t read_size = fread(log_contents, 1, sizeof(log_contents) - 1, fp);
    TEST_ASSERT(read_size > 0); // Ensure we read something
    log_contents[read_size] = '\0';
    fclose(fp);

    TEST_ASSERT(strcmp(log_contents, "helloworld") == 0);

    /* Clean up */
    unlink(logfile);
}

TEST_LIST = {
    {"open", test_open},
    {"loopback", test_loopback},
    {"loopback_random", test_loopback_random},
    {"read_line", test_read_line},
    {"read_timeout", test_read_timeout},
    {"logfile", test_logfile},
    {NULL, NULL},
};