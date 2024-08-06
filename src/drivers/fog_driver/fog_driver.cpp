#include "fog_driver.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <cmath>

#define DEG_TO_RAD ((float)M_PI / 180.0f)
#define _device_id uint32_t(4325386)

FOGDriver::FOGDriver() : ModuleBase(), uart_fd(-1), _gyro_pub(nullptr)
{
    // Open UART
    uart_fd = ::open("/dev/ttyS1", O_RDWR | O_NOCTTY);
    if (uart_fd < 0) {
        PX4_ERR("Failed to open UART device");
        return;
    }

    // Configure UART
    struct termios uart_config;
    int termios_state;
    if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
        PX4_ERR("Error getting UART config: %d", termios_state);
        return;
    }
    uart_config.c_oflag &= ~ONLCR;
    if ((termios_state = cfsetispeed(&uart_config, B921600)) < 0) {
        PX4_ERR("Error setting UART input speed: %d", termios_state);
        return;
    }
    if ((termios_state = cfsetospeed(&uart_config, B921600)) < 0) {
        PX4_ERR("Error setting UART output speed: %d", termios_state);
        return;
    }
    if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_ERR("Error setting UART config: %d", termios_state);
        return;
    }
}

FOGDriver::~FOGDriver()
{
    if (uart_fd >= 0) {
        ::close(uart_fd);
    }
}

void FOGDriver::run()
{
    while (!should_exit()) {
        readData();
    }
}

void FOGDriver::readData()
{
    uint8_t GYRO_QUEUE_SIZE = 8;
    uint8_t buffer[60]; // Buffer to hold start byte and 9 data bytes
    FOG_DATA_Type data;

    int bytes_read = ::read(uart_fd, buffer, sizeof(buffer));
    hrt_abstime timestamp_sample = hrt_absolute_time(); // Capture timestamp after reading

    int index = -1;
    for(int i=0; i< bytes_read - 9;i++){
        if(buffer[i] == 0x80){
            index = i;
            break;
        }
    }

    if(index == -1){
        return;
    }

    if (buffer[index] == 0x80) {
        // Start byte is correct, process the data
        data.x = ((uint32_t)buffer[index+1] << 16) | ((uint32_t)buffer[index+2] << 8) | ((uint32_t)buffer[index+3]);
        data.y = ((int32_t)buffer[index+4] << 16) | ((int32_t)buffer[index+5] << 8) | buffer[index+6];
        data.z = ((int32_t)buffer[index+7] << 16) | ((int32_t)buffer[index+8] << 8) | buffer[index+9];

        // Sign extension
        if (data.x & 0x800000) data.x |= 0xFF000000;
        if (data.y & 0x800000) data.y |= 0xFF000000;
        if (data.z & 0x800000) data.z |= 0xFF000000;

        sensor_gyro_s sensor_gyro{};
        sensor_gyro.timestamp_sample = timestamp_sample;
        sensor_gyro.timestamp = hrt_absolute_time();
        sensor_gyro.device_id = _device_id;
        sensor_gyro.x = ((float)data.z / 3600.0f) * DEG_TO_RAD;
        sensor_gyro.y = -((float)data.y / 3600.0f) * DEG_TO_RAD;
        sensor_gyro.z = ((float)data.x / 3600.0f) * DEG_TO_RAD;
        sensor_gyro.temperature = 30.0f;
        sensor_gyro.samples = 1;

        if (_gyro_pub == nullptr) {
            _gyro_pub = orb_advertise_queue(ORB_ID(sensor_gyro), &sensor_gyro, GYRO_QUEUE_SIZE);
        } else {
            orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &sensor_gyro);
            // orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &sensor_gyro);
        }
    }
}

int FOGDriver::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("fog_driver",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2000,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);
    if (_task_id < 0) {
        PX4_ERR("task start failed");
        return -errno;
    }
    return 0;
}

FOGDriver *FOGDriver::instantiate(int argc, char *argv[])
{
    return new FOGDriver();
}

int FOGDriver::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Module to read data from FOG sensor and publish it to uORB topic.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("fog_driver", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int FOGDriver::custom_command(int argc, char *argv[])
{
    // Custom command handling can be added here if needed
    return print_usage("Unknown command");
}

extern "C" __EXPORT int fog_driver_main(int argc, char *argv[])
{
    return FOGDriver::main(argc, argv);
}
