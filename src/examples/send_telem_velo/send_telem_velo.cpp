// src/examples/send_telem_velo/send_telem_velo.cpp
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

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_angular_velocity.h>


const float M_PI = 3.14159265358979323846;
const float two_M_PI = 6.283185307179586;

extern "C" __EXPORT int send_telem_velo_main(int argc, char *argv[]);

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

packetType angularRate;

static bool _task_should_exit = false;
static bool _is_running = false;
static int _send_telem_velo_task = -1;

void send_velocity_usage()
{
    PX4_INFO("Usage: send_telem_velo {start|stop|status}");
}

int send_telem_velo_thread_main(int argc, char *argv[])
{
    PX4_INFO("Starting send_telem_velo thread");

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

    cfsetispeed(&uart_config, B115200);
    cfsetospeed(&uart_config, B115200);

    tcsetattr(uart_fd, TCSANOW, &uart_config);

    angularRate.header = 0xAAAA;
    angularRate.commandCode = 0x0001;
    angularRate.checksum = 0x00;
    angularRate.footer = 0x5555;

    int vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    struct vehicle_angular_velocity_s angular_velocity;

    _is_running = true;


    uint8_t *ptr = (uint8_t*)&angularRate;
    while (!_task_should_exit) {
        bool updated;
        orb_check(vehicle_angular_velocity_sub, &updated);
        if (updated) {
            // Copy data to the struct
            orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_sub, &angular_velocity);

            // Update angularRate with new data
            angularRate.x =  ((float)angular_velocity.xyz[0]/two_M_PI)*524288.0f;
            angularRate.y = ((float)angular_velocity.xyz[1]/two_M_PI)*524288.0f;
            angularRate.z = ((float)angular_velocity.xyz[2]/two_M_PI)*524288.0f;

            // Calculate checksum
            angularRate.checksum = 0x0000;

            for(int i = 2; i < 16; i++) 
            {
                angularRate.checksum ^= ptr[i];
            }

            write(uart_fd, &angularRate, sizeof(angularRate));
        }
        // usleep(500000);
    }

    close(uart_fd);
    PX4_INFO("send_telem_velo thread finished");
    _is_running = false;

    return 0;
}

int send_telem_velo_main(int argc, char *argv[])
{
    if (argc < 2) {
        send_velocity_usage();
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (_is_running) {
            PX4_WARN("send_telem_velo already running");
            return 0;
        }

        _task_should_exit = false;
        _send_telem_velo_task = px4_task_spawn_cmd("send_telem_velo",
                                                 SCHED_DEFAULT,
                                                 SCHED_PRIORITY_DEFAULT,
                                                 2000,
                                                 send_telem_velo_thread_main,
                                                 (char *const *)argv);
        if (_send_telem_velo_task < 0) {
            PX4_ERR("Failed to start send_telem_velo");
            return -1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!_is_running) {
            PX4_WARN("send_telem_velo not running");
            return 0;
        }

        _task_should_exit = true;

        while (_is_running) {
            usleep(100000);
        }

        PX4_INFO("send_telem_velo stopped");
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (_is_running) {
            PX4_INFO("send_telem_velo is running");
        } else {
            PX4_INFO("send_telem_velo is not running");
        }

        return 0;
    }

    send_velocity_usage();
    return 1;
}
