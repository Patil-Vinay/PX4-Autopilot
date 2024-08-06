#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <nuttx/can.h>
#include <netpacket/can.h>
#include <sys/socket.h>
#include <net/if.h>  // Add this for struct ifreq
#include <fcntl.h>
#include <string.h>
#include <errno.h>

extern "C" __EXPORT int can_frame_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static int daemon_task = -1;

static int can_frame_thread_main(int argc, char *argv[])
{
    int s,t;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

     PX4_INFO("can_frame_thread_main started");

    // Open CAN socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        PX4_ERR("Error while opening socket: %s", strerror(errno));
        return -1;
    }
    else
    {
	PX4_INFO("Socket Opened: %s", strerror(errno));
    }

    strncpy(ifr.ifr_name, "can1", IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    t = ioctl(s, SIOCGIFINDEX, &ifr);
    if (t < 0) {
        PX4_ERR("ioctl error: %s", strerror(errno));
        close(s);
        return -1;
    }
    else
    {
	PX4_INFO("Can0 Opened: %s", strerror(errno));
    }

    PX4_INFO("Socket Opened: %s", strerror(errno));

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        PX4_ERR("Error in socket bind: %s", strerror(errno));
        close(s);
        return -2;
    }

    while (1) {
        // Send CAN frame
        // PX4_INFO("Raw CAN Frame Data: ");
        // for (size_t i = 0; i < sizeof(frame); ++i) {
        //     PX4_INFO("%02X ", reinterpret_cast<uint8_t*>(&frame)[i]);
        // }
        // PX4_INFO(""); // New line after the raw data output
        // for (int i = 1584; i < 1700; ++i) {
        frame.can_id =  0x700;
        frame.can_dlc = 6;
        frame.data[0] = 0xAA;
        frame.data[1] = 0xBB;
        frame.data[2] = 0xCC;
        frame.data[3] = 0xDD;
        frame.data[4] = 0xEE;
        frame.data[5] = 0xFF;
    // frame.data[6] = 0x11;
    // frame.data[7] = 0x22;
        ssize_t nbytes = write(s, &frame, sizeof(struct can_frame));
        // PX4_INFO("Here 2");
        if (nbytes != sizeof(struct can_frame)) {
            PX4_ERR("Error in socket write: %s", strerror(errno));
        } else {
            PX4_INFO("Message sent on CAN bus");
        }
	    // PX4_INFO("Print hotay, pan data nahi jaat");
        // Sleep for 1 second
        px4_usleep(2000);
        }

    close(s);
    return 0;
}

static void usage(const char *reason)
{
    if (reason) {
        PX4_ERR("%s", reason);
    }
    PX4_INFO("usage: can_frame {start|stop|status}");
}

extern "C" __EXPORT int can_frame_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (daemon_task >= 0) {
            PX4_INFO("can_frame already running");
            return 0;
        }

        daemon_task = px4_task_spawn_cmd("can_frame",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2000,
                                         (px4_main_t)can_frame_thread_main,
                                         (char *const *)argv);

        if (daemon_task < 0) {
            PX4_ERR("failed to start can_frame task");
            return -1;
        }

        PX4_INFO("can_frame started successfully");
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (daemon_task < 0) {
            PX4_INFO("can_frame is not running");
            return 0;
        }

        thread_should_exit = true;
        px4_usleep(200000); // Give some time for the thread to exit
        if (px4_task_delete(daemon_task) == 0) {
            daemon_task = -1;
            PX4_INFO("can_frame stopped");
        } else {
            PX4_ERR("failed to stop can_frame task");
            return -1;
        }
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (daemon_task < 0) {
            PX4_INFO("can_frame is not running");
        } else {
            PX4_INFO("can_frame is running");
        }
        return 0;
    }

    usage("unrecognized command");
    return 1;
}
