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
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>

const float One_M_PI = 3.14159265358979323846;
const float two_M_PI = 6.283185307179586;

extern "C" __EXPORT int send_telem_angle_main(int argc, char *argv[]);

typedef struct
{
    uint16_t header;
    uint16_t commandCode;
    float roll;
    float pitch;
    float yaw;
    uint16_t checksum;
    uint16_t footer;
} packetType;

packetType attitude;

static bool _task_should_exit = false;
static bool _is_running = false;
static int _send_telem_angle_task = -1;

void send_angle_usage()
{
    PX4_INFO("Usage: send_telem_angle {start|stop|status}");
}

void quaternion_to_euler(const matrix::Quatf &q, float &roll, float &pitch, float &yaw)
{
    matrix::Eulerf euler(q);
    roll = euler.phi();
    pitch = euler.theta();
    yaw = euler.psi();
}

int send_telem_angle_thread_main(int argc, char *argv[])
{
    PX4_INFO("Starting send_telem_angle thread");

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

    cfsetispeed(&uart_config, B921600);
    cfsetospeed(&uart_config, B921600);

    tcsetattr(uart_fd, TCSANOW, &uart_config);

    attitude.header = 0xAAAA;
    attitude.commandCode = 0x0001;
    attitude.checksum = 0x00;
    attitude.footer = 0x5555;

    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    struct vehicle_attitude_s attitude_data;

    _is_running = true;

    uint8_t *ptr = (uint8_t*)&attitude;
    while (!_task_should_exit) {
        bool updated;
        orb_check(vehicle_attitude_sub, &updated);
        if (updated) {
            // Copy data to the struct
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude_data);

            float roll, pitch, yaw;
            matrix::Quatf q(attitude_data.q);
            quaternion_to_euler(q, roll, pitch, yaw);

            // Update angularRate with new data
            attitude.roll =  (roll/two_M_PI)*524288.0f;
            attitude.pitch = (pitch/two_M_PI)*524288.0f;
            attitude.yaw = (yaw/two_M_PI)*524288.0f;

            // Calculate checksum
            attitude.checksum = 0x0000;

            for(int i = 2; i < 16; i++)
            {
                attitude.checksum ^= ptr[i];
            }
            usleep(20000);
            write(uart_fd, &attitude, sizeof(attitude));
        }
    }

    close(uart_fd);
    PX4_INFO("send_telem_angle thread finished");
    _is_running = false;

    return 0;
}

int send_telem_angle_main(int argc, char *argv[])
{
    if (argc < 2) {
        send_angle_usage();
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (_is_running) {
            PX4_WARN("send_telem_angle already running");
            return 0;
        }

        _task_should_exit = false;
        _send_telem_angle_task = px4_task_spawn_cmd("send_telem_angle",
                                                 SCHED_DEFAULT,
                                                 SCHED_PRIORITY_DEFAULT,
                                                 2000,
                                                 send_telem_angle_thread_main,
                                                 (char *const *)argv);
        if (_send_telem_angle_task < 0) {
            PX4_ERR("Failed to start send_telem_angle");
            return -1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!_is_running) {
            PX4_WARN("send_telem_angle not running");
            return 0;
        }

        _task_should_exit = true;

        while (_is_running) {
            usleep(100000);
        }

        PX4_INFO("send_telem_angle stopped");
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (_is_running) {
            PX4_INFO("send_telem_angle is running");
        } else {
            PX4_INFO("send_telem_angle is not running");
        }

        return 0;
    }

    send_angle_usage();
    return 1;
}
