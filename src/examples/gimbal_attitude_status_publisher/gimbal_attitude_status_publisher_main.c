#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <unistd.h>
#include <math.h>

__EXPORT int gimbal_attitude_status_publisher_main(int argc, char *argv[]);

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int daemon_task; /**< Handle of deamon task / thread */

int gimbal_attitude_status_publisher_thread_main(int argc, char *argv[]);

int gimbal_attitude_status_publisher_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_ERR("Usage: gimbal_attitude_status_publisher {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_WARN("already running");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("gimbal_attitude_status_publisher",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2000,
                                         gimbal_attitude_status_publisher_thread_main,
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

int gimbal_attitude_status_publisher_thread_main(int argc, char *argv[])
{
    PX4_INFO("Gimbal Attitude Status Publisher started");

    thread_running = true;

    struct gimbal_device_attitude_status_s gimbal_device_attitude_status;
    memset(&gimbal_device_attitude_status, 0, sizeof(gimbal_device_attitude_status));
    orb_advert_t gimbal_device_attitude_status_pub = orb_advertise(ORB_ID(gimbal_device_attitude_status), &gimbal_device_attitude_status);

    if (gimbal_device_attitude_status_pub == NULL) {
        PX4_ERR("Failed to advertise gimbal_device_attitude_status topic");
        thread_running = false;
        return -1;
    }

    while (!thread_should_exit) {
        // Generate some dummy data
        gimbal_device_attitude_status.timestamp = hrt_absolute_time();
        gimbal_device_attitude_status.q[0] = cos(gimbal_device_attitude_status.timestamp / 1000000.0);
        gimbal_device_attitude_status.q[1] = sin(gimbal_device_attitude_status.timestamp / 1000000.0);
        gimbal_device_attitude_status.q[2] = 0.0;
        gimbal_device_attitude_status.q[3] = 0.0;

        orb_publish(ORB_ID(gimbal_device_attitude_status), gimbal_device_attitude_status_pub, &gimbal_device_attitude_status);
        PX4_INFO("Publishing Gimbal Attitude: Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
                 (double)gimbal_device_attitude_status.q[0],
                 (double)gimbal_device_attitude_status.q[1],
                 (double)gimbal_device_attitude_status.q[2]);

        usleep(100000); // Sleep for 100ms
    }

    orb_unadvertise(gimbal_device_attitude_status_pub);

    PX4_INFO("Gimbal Attitude Status Publisher exiting");
    thread_running = false;

    return 0;
}
