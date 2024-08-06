#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <unistd.h>

__EXPORT int gimbal_attitude_status_main(int argc, char *argv[]);

static bool thread_should_exit = false; /**< Daemon exit flag */
static bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */

/**
 * Mainloop of daemon.
 */
int gimbal_attitude_status_thread_main(int argc, char *argv[]);

int gimbal_attitude_status_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_ERR("Usage: gimbal_attitude_status {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_WARN("already running");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("gimbal_attitude_status",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2000,
                                         gimbal_attitude_status_thread_main,
                                         (char *const *)argv);

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            PX4_INFO("running");
        } else {
            PX4_INFO("not started");
        }

        return 0;
    }

    PX4_ERR("unrecognized command");
    return 1;
}

int gimbal_attitude_status_thread_main(int argc, char *argv[])
{
    PX4_INFO("Gimbal Attitude Status module started");

    thread_running = true;

    int gimbal_device_attitude_status_sub = orb_subscribe(ORB_ID(gimbal_device_attitude_status));
    if (gimbal_device_attitude_status_sub < 0) {
        PX4_ERR("Failed to subscribe to gimbal_device_attitude_status topic");
        thread_running = false;
        return -1;
    }

    struct gimbal_device_attitude_status_s gimbal_device_attitude_status;

    while (!thread_should_exit) {
        bool updated;
        orb_check(gimbal_device_attitude_status_sub, &updated);
        PX4_INFO("Checking for updates on gimbal_device_attitude_status topic");

        if (updated) {
            PX4_INFO("Update found on gimbal_device_attitude_status topic");
            orb_copy(ORB_ID(gimbal_device_attitude_status), gimbal_device_attitude_status_sub, &gimbal_device_attitude_status);

            // Print the angular velocities directly using the  correct member names
            PX4_INFO("Gimbal Rates: Roll Rate: %.2f deg/s, Pitch Rate: %.2f deg/s, Yaw Rate: %.2f deg/s",
                     (double)gimbal_device_attitude_status.angular_velocity_x,
                     (double)gimbal_device_attitude_status.angular_velocity_y,
                     (double)gimbal_device_attitude_status.angular_velocity_z);
        } else {
            PX4_INFO("No update on gimbal_device_attitude_status topic");
        }

        usleep(100000); // Sleep for 100ms
    }

    orb_unsubscribe(gimbal_device_attitude_status_sub);

    PX4_INFO("Gimbal Attitude Status module exiting");
    thread_running = false;

    return 0;
}
