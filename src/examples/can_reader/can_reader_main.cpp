#include <nuttx/can.h>
#include <sys/socket.h>
#include <netpacket/can.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstdio>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#define CAN_INTERFACE "can0"

extern "C" __EXPORT int CanReader_main(int argc, char *argv[]);

int CanReader_main(int argc, char *argv[]) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    int sockfd;

    // Create a socket
    if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        PX4_ERR("Socket creation failed: %d", errno);
        return 1;
    }

    // Specify the CAN interface to use
    strcpy(ifr.ifr_name, CAN_INTERFACE);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        PX4_ERR("ioctl failed: %d", errno);
        return 1;
    }

    // Bind the socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        PX4_ERR("Bind failed: %d", errno);
        return 1;
    }

    PX4_INFO("CAN Reader started on %s", CAN_INTERFACE);

    // Read CAN messages
    while (true) {
        int nbytes = read(sockfd, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            PX4_ERR("Read failed: %d", errno);
            break;
        } else if (nbytes == sizeof(struct can_frame)) {
            // Print the received CAN message
            PX4_INFO("Received CAN message: ID = 0x%03lx DLC = %d Data =", (unsigned long)frame.can_id, frame.can_dlc);
            for (int i = 0; i < frame.can_dlc; ++i) {
                PX4_INFO(" 0x%02x", frame.data[i]);
            }
        }
    }

    close(sockfd);
    return 0;
}
