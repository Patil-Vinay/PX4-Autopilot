// src/examples/send_integers/send_integers.cpp
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/log.h>

extern "C" __EXPORT int send_integers_main(int argc, char *argv[]);

typedef struct
{
    uint16_t header;
    uint16_t commandCode;
    float x;
    float y;
    float z;
    uint16_t checksum;
    uint16_t footer;
} packetType;

packetType static_angularRate, static_attitude;

static bool _task_should_exit = false;
static bool _is_running = false;
static int _send_integers_task = -1;

void send_integer_usage()
{
    PX4_INFO("Usage: send_integers {start|stop|status}");
}

int send_integers_thread_main(int argc, char *argv[])
{
    PX4_INFO("Starting send_integers thread");

    // Open the TELEM3 port
    int uart_fd = open("/dev/ttyS1", O_WRONLY | O_NOCTTY);
    if (uart_fd < 0) {
        PX4_ERR("Failed to open UART port");
        return -1;
    }

    // Configure UART
    struct termios uart_config;
    tcgetattr(uart_fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR; // No CR for every LF
    uart_config.c_cflag |= (CLOCAL | CREAD);
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag &= ~PARENB;
    uart_config.c_cflag &= ~CSTOPB;
    uart_config.c_cflag &= ~CRTSCTS;

    cfsetispeed(&uart_config, B57600);
    cfsetospeed(&uart_config, B57600);

    tcsetattr(uart_fd, TCSANOW, &uart_config);

    float x = 240;
    float y = 120;
    float z = 60;
    static_angularRate.header = 0xAAAA;
    static_angularRate.commandCode = 0x0001;
    static_angularRate.x = x;
    static_angularRate.y = y;
    static_angularRate.z = z;
    static_angularRate.checksum = 0x00;
    static_angularRate.footer = 0x5555;

    // uint8_t *ptr = (uint8_t*)&static_angularRate;

    // for(int i = 0; i < sizeof(static_angularRate); i++)
    // {
    //     PX4_INFO("0x%02X", ptr[i]);
    // }

    _is_running = true;

    while (!_task_should_exit) {
        write(uart_fd, &static_angularRate, sizeof(static_angularRate));
        usleep(500000);
    }

    close(uart_fd);
    PX4_INFO("send_integers thread finished");
    _is_running = false;

    return 0;
}

int send_integers_main(int argc, char *argv[])
{
    if (argc < 2) {
        send_integer_usage();
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (_is_running) {
            PX4_WARN("send_integers already running");
            return 0;
        }

        _task_should_exit = false;
        _send_integers_task = px4_task_spawn_cmd("send_integers",
                                                 SCHED_DEFAULT,
                                                 SCHED_PRIORITY_DEFAULT,
                                                 2000,
                                                 send_integers_thread_main,
                                                 (char *const *)argv);
        if (_send_integers_task < 0) {
            PX4_ERR("Failed to start send_integers");
            return -1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!_is_running) {
            PX4_WARN("send_integers not running");
            return 0;
        }

        _task_should_exit = true;

        while (_is_running) {
            usleep(100000);
        }

        PX4_INFO("send_integers stopped");
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (_is_running) {
            PX4_INFO("send_integers is running");
        } else {
            PX4_INFO("send_integers is not running");
        }

        return 0;
    }

    send_integer_usage();
    return 1;
}
