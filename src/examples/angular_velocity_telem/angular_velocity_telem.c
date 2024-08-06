#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <lib/mathlib/mathlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#define TELEM2_DEVICE "/dev/ttyS1"
#define BAUDRATE B115200

__EXPORT int angular_velocity_telem_main(int argc, char *argv[]);

int open_telem2_port(void) {
    int fd = open(TELEM2_DEVICE, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        PX4_ERR("Failed to open TELEM2 port");
        return -1;
    }

    struct termios config;
    if (tcgetattr(fd, &config) < 0) {
        PX4_ERR("Failed to get current port configuration");
        close(fd);
        return -1;
    }

    config.c_iflag = 0;
    config.c_oflag = 0;
    config.c_lflag = 0;
    config.c_cflag = CS8 | CREAD | CLOCAL;

    cfsetispeed(&config, BAUDRATE);
    cfsetospeed(&config, BAUDRATE);

    if (tcsetattr(fd, TCSANOW, &config) < 0) {
        PX4_ERR("Failed to set port configuration");
        close(fd);
        return -1;
    }

    return fd;
}

int angular_velocity_telem_main(int argc, char *argv[]) {
    PX4_INFO("Starting Angular Velocity Telemetry Module...");

    int sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    orb_set_interval(sub, 100); // Set the update interval to 100ms

    struct vehicle_angular_velocity_s angular_velocity;

    int telem2_fd = open_telem2_port();
    if (telem2_fd == -1) {
        return -1;
    }

    while (true) {
        bool updated;
        orb_check(sub, &updated);

        if (updated) {
            orb_copy(ORB_ID(vehicle_angular_velocity), sub, &angular_velocity);
            PX4_INFO("Angular Velocity: [%.2f, %.2f, %.2f]",
                     (double)angular_velocity.xyz[0], (double)angular_velocity.xyz[1], (double)angular_velocity.xyz[2]);

            // Prepare the data to be sent
            char buffer[100];
            int n = snprintf(buffer, sizeof(buffer), "Angular Velocity: [%.2f, %.2f, %.2f]\n",
                             (double)angular_velocity.xyz[0], (double)angular_velocity.xyz[1], (double)angular_velocity.xyz[2]);

            // Send the data over TELEM2
            if (write(telem2_fd, buffer, n) < 0) {
                PX4_ERR("Failed to write to TELEM2 port");
            }
        }

        px4_usleep(100000); // Sleep for 100ms
    }

    close(telem2_fd);
    return 0;
}

