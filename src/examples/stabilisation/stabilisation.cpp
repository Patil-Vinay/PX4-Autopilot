#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <nuttx/can.h>
#include <netpacket/can.h>
#include <drivers/drv_hrt.h>
#include <sys/socket.h>
#include <lib/perf/perf_counter.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <matrix/math.hpp>
#include <inttypes.h>


const float One_M_PI = 3.14159265358979323846;
const float two_M_PI = 6.283185307179586;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} packetTypeAtt;

typedef struct {
    float x;
    float y;
    float z;
} packetType;

#define CMD_IDX_OPERATION_MODE 0x4E
#define CMD_IDX_MOTION_MODE 0x8D
#define CMD_IDX_PROFILE_ACCELERATION 0x88
#define CMD_IDX_PROFILE_DECELERATION 0x89
#define CMD_IDX_PROFILE_SPEED 0x8A
#define CMD_IDX_ENABLE_MOTOR 0x100
#define CMD_IDX_RELATIVE_POSITION 0x87
#define CMD_IDX_TARGET_ABSOLUTE_POSITION 0x86
#define CMD_IDX_START_MOTION 0x83
#define CMD_IDX_STOP_MOTION 0x84
#define CMD_IDX_ACTUAL_POSITION 0x02

uint16_t TILT_CAN_ID = 0x11;
uint16_t PAN_CAN_ID = 0x12;

struct CANCommand {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    hrt_abstime timestamp;
};

CANCommand lastSentCommand;
bool expectingResponse = false;
const int TIMEOUT_MS = 2000;

class CANHandler {
public:
    CANHandler() : s(-1), vehicle_attitude_sub(-1), vehicle_angular_velocity_sub(-1), mode(0), current_position_tilt(0), current_position_pan(0), runLoop(true), positionRead(false) {}

    ~CANHandler() {
        closeSocket();
    }

    bool initSocket() {
        if (s >= 0) {
            return true;
        }

        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) {
            PX4_ERR("Error while opening socket: %s", strerror(errno));
            return false;
        }

        struct ifreq ifr;
        strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
            PX4_ERR("ioctl error: %s", strerror(errno));
            closeSocket();
            return false;
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            PX4_ERR("Error in socket bind: %s", strerror(errno));
            closeSocket();
            return false;
        }

        int flags = fcntl(s, F_GETFL, 0);
        if (flags == -1) {
            PX4_ERR("Error getting socket flags: %s", strerror(errno));
            closeSocket();
            return false;
        }

        flags &= ~O_NONBLOCK;
        if (fcntl(s, F_SETFL, flags) == -1) {
            PX4_ERR("Error setting socket to blocking mode: %s", strerror(errno));
            closeSocket();
            return false;
        }

        PX4_INFO("CAN socket initialized and bound to can0");
        return true;
    }

    void closeSocket() {
        PX4_INFO("closeSocket");
        if (s >= 0) {
            close(s);
            s = -1;
        }
    }

    void setArguments(int argc, char *argv[]) {
        arg_count = argc;
        for (int i = 0; i < argc; ++i) {
            strncpy(arguments[i], argv[i], sizeof(arguments[i]) - 1);
            arguments[i][sizeof(arguments[i]) - 1] = '\0';
        }
    }

    int getArgCount() const {
        return arg_count;
    }

    const char* getArgument(int index) const {
        return arguments[index];
    }

    void sendCANCommand(const CANCommand &command) {
        if (s < 0) {
            PX4_ERR("Invalid socket descriptor");
            return;
        }

        struct can_frame frame;
        frame.can_id = command.id;
        frame.can_dlc = command.length;
        for (int i = 0; i < command.length; ++i) {
            frame.data[i] = command.data[i];
        }
        ssize_t nbytes = write(s, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) {
            PX4_ERR("Error in socket write: %s", strerror(errno));
        }

        lastSentCommand = command;
        lastSentCommand.timestamp = hrt_absolute_time();
    }

    void handleReceivedFrame(const struct can_frame &message) {
        uint32_t expectedResponseId = 0x5D1;
        PX4_INFO("Received CAN frame: id=0x%" PRIx32 ", dlc=%d", message.can_id, message.can_dlc);
        PX4_INFO("Data: %02X %02X %02X %02X %02X",message.data[0], message.data[1], message.data[2], message.data[3], message.data[4]);

        if (message.can_id == expectedResponseId) {
            PX4_INFO("Expected response received");
            if (message.data[message.can_dlc - 1] == 0x3E) {
                PX4_INFO("Operation successful");
                int32_t current_position = (message.data[0] << 24) | (message.data[1] << 16) | (message.data[2] << 8) | message.data[3];
                PX4_INFO("Current position read: %ld", current_position);
                float current_position_degrees = (current_position / 524288.0f) * 360;
                PX4_INFO("Current position in degrees: %f", current_position_degrees);
                if (message.can_id == TILT_CAN_ID) {
                    current_position_tilt = current_position;
                } else if (message.can_id == PAN_CAN_ID) {
                    current_position_pan = current_position;
                }
                positionRead = false;
            } else if (message.data[message.can_dlc - 1] == 0x80) {
                PX4_WARN("Operation failed");
            } else {
                PX4_WARN("Unexpected response data: 0x%02X", message.data[message.can_dlc - 1]);
            }
        } else {
            PX4_WARN("Unexpected message received: id=0x%03X", (unsigned int)message.can_id);
        }
        expectingResponse = false;
    }

    void sendCANCommandAndWait(const CANCommand &command) {
        sendCANCommand(command);
        hrt_abstime startTime = hrt_absolute_time();
        struct pollfd fds;
        fds.fd = s;
        fds.events = POLLIN;

        while (expectingResponse && (hrt_elapsed_time(&startTime) < TIMEOUT_MS)) {
            int ret = poll(&fds, 1, TIMEOUT_MS);
            if (ret > 0) {
                if (fds.revents & POLLIN) {
                    struct can_frame message;
                    ssize_t nbytes = read(s, &message, sizeof(struct can_frame));
                    if (nbytes == sizeof(struct can_frame)) {
                        PX4_INFO("CAN frame received");
                        handleReceivedFrame(message);
                    } else {
                        PX4_WARN("Incomplete CAN frame read");
                    }
                } else {
                    PX4_WARN("POLLIN not set");
                }
            } else if (ret == 0) {
                PX4_WARN("Poll timeout");
                expectingResponse = false;
            } else {
                PX4_ERR("Poll error: %s", strerror(errno));
            }
            px4_usleep(1000);
        }
    }

    void sendReadDataCommandAndWait(uint16_t driveID, uint16_t commandIndex, uint16_t subIndex) {
        CANCommand command;
        command.id = 0x640 + driveID;
        command.length = (subIndex == 0) ? 2 : 4;
        command.data[0] = (commandIndex >> 8) & 0xFF;
        command.data[1] = commandIndex & 0xFF;
        if (subIndex != 0) {
            command.data[2] = (subIndex >> 8) & 0xFF;
            command.data[3] = subIndex & 0xFF;
        }
        sendCANCommandAndWait(command);
    }

    void sendWriteDataCommandAndWait(uint16_t driveID, uint16_t commandIndex, uint32_t value, uint16_t subIndex) {
        CANCommand command;
        command.id = 0x640 + driveID;
        command.length = (subIndex == 0) ? 6 : 8;
        command.data[0] = (commandIndex >> 8) & 0xFF;
        command.data[1] = commandIndex & 0xFF;
        if (subIndex != 0) {
            command.data[2] = (subIndex >> 8) & 0xFF;
            command.data[3] = subIndex & 0xFF;
        }
        command.data[command.length - 4] = (value >> 24) & 0xFF;
        command.data[command.length - 3] = (value >> 16) & 0xFF;
        command.data[command.length - 2] = (value >> 8) & 0xFF;
        command.data[command.length - 1] = value & 0xFF;
        sendCANCommandAndWait(command);
    }

    void readActualPosition(uint16_t driveID) {
        expectingResponse = true;
        sendReadDataCommandAndWait(driveID, CMD_IDX_ACTUAL_POSITION, 0);
    }

    void writeOperationMode(uint16_t driveID, uint16_t operation_mode) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_OPERATION_MODE, operation_mode, 0);
    }

    void writeMotionMode(uint16_t driveID, uint32_t motion_mode) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_MOTION_MODE, motion_mode, 0);
    }

    void writeProfileAcceleration(uint16_t driveID, uint32_t acceleration) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_PROFILE_ACCELERATION, acceleration, 0);
    }

    void writeProfileDeceleration(uint16_t driveID, uint32_t deceleration) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_PROFILE_DECELERATION, deceleration, 0);
    }

    void writeProfileSpeed(uint16_t driveID, uint32_t speed) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_PROFILE_SPEED, speed, 0);
    }

    void enableMotor(uint16_t driveID, bool enable_disable) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_ENABLE_MOTOR, enable_disable, 0);
    }

    void writeRelativePosition(uint16_t driveID, uint32_t position) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_RELATIVE_POSITION, position, 0);
    }

    void writeTargetAbsolutePosition(uint16_t driveID, uint32_t position) {
        sendWriteDataCommandAndWait(driveID, CMD_IDX_TARGET_ABSOLUTE_POSITION, position, 0);
    }

    void startMotion(uint16_t driveID) {
        CANCommand command;
        command.id = 0x640 + driveID;
        command.length = 2;
        command.data[1] = CMD_IDX_START_MOTION;
        command.data[0] = 0;

        sendCANCommand(command);
    }

    void stopMotion(uint16_t driveID) {
        CANCommand command;
        command.id = 0x640 + driveID;
        command.length = 2;
        command.data[1] = CMD_IDX_STOP_MOTION;
        command.data[0] = 0;

        sendCANCommand(command);
    }

    void can_quaternion_to_euler(const matrix::Quatf &q, float &roll, float &pitch, float &yaw) {
        matrix::Eulerf euler(q);
        roll = euler.phi();
        pitch = euler.theta();
        yaw = euler.psi();
    }

    void setMode(int new_mode) {
        mode = new_mode;
    }

    int getMode() const {
        return mode;
    }

    void subscribeTopics() {
        vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    }

    void processAttitude() {
        bool updated;
        orb_check(vehicle_attitude_sub, &updated);
        if (updated) {
            struct vehicle_attitude_s attitude_data;
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude_data);

            float roll, pitch, yaw;
            matrix::Quatf q(attitude_data.q);
            can_quaternion_to_euler(q, roll, pitch, yaw);

            float attitude_roll = (roll / two_M_PI) * 524288.0f;
            float attitude_pitch = (pitch / two_M_PI) * 524288.0f;
            float attitude_yaw = (yaw / two_M_PI) * 524288.0f;

            (void)attitude_roll;
            (void)attitude_yaw;

            float argangle = atof(getArgument(3));
            int32_t target_position_tilt = (int32_t)((argangle / 360.0f) * 524288.0f);
            int32_t target_position_pan = (int32_t)((argangle / 360.0f) * 524288.0f);
            sendWriteDataCommandAndWait(TILT_CAN_ID, CMD_IDX_TARGET_ABSOLUTE_POSITION, target_position_tilt + (int32_t)attitude_pitch, 0);
            usleep(200);
            startMotion(TILT_CAN_ID);
            sendWriteDataCommandAndWait(PAN_CAN_ID, CMD_IDX_TARGET_ABSOLUTE_POSITION, target_position_pan + (int32_t)attitude_yaw, 0);
            usleep(200);
            startMotion(PAN_CAN_ID);
        }
    }

    void processAngularVelocity() {
        bool updated;
        orb_check(vehicle_angular_velocity_sub, &updated);
        if (updated) {
            struct vehicle_angular_velocity_s angular_velocity;
            orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_sub, &angular_velocity);

            angularRate.x = ((float)angular_velocity.xyz[0] / two_M_PI) * 524288.0f;
            angularRate.y = ((float)angular_velocity.xyz[1] / two_M_PI) * 524288.0f;
            angularRate.z = ((float)angular_velocity.xyz[2] / two_M_PI) * 524288.0f;

            sendWriteDataCommandAndWait(TILT_CAN_ID, CMD_IDX_PROFILE_SPEED, (int32_t)angularRate.y, 0);
            usleep(200);
            sendWriteDataCommandAndWait(PAN_CAN_ID, CMD_IDX_PROFILE_SPEED, (int32_t)angularRate.z, 0);
            usleep(200);
        }
    }

    void process() {
        PX4_INFO("process");
        while (!thread_should_exit && runLoop) {
            if (mode == 0) {
                processAttitude();
            } else if (mode == 1) {
                processAngularVelocity();
            }
        }
    }

    static bool thread_should_exit;

    int32_t getCurrentPosition(uint16_t driveID) const {
        if (driveID == TILT_CAN_ID) {
            PX4_INFO("Function Current position: %ld", (int32_t)current_position_tilt);
            return current_position_tilt;
        } else if (driveID == PAN_CAN_ID) {
            PX4_INFO("Function Current position: %ld", (int32_t)current_position_pan);
            return current_position_pan;
        }
        return 0;
    }

    void setRunLoop(bool run) {
        runLoop = run;
    }

private:
    int s;
    int vehicle_attitude_sub;
    int vehicle_angular_velocity_sub;
    int mode;
    int32_t current_position_tilt;
    int32_t current_position_pan;
    bool runLoop;
    bool positionRead;
    packetType angularRate;
    char arguments[10][100];
    int arg_count;
};

bool CANHandler::thread_should_exit = false;

static CANHandler can_handler;
static bool _is_running = false;
static int daemon_task = -1;

static int stabilisation_thread_main(int argc, char *argv[]) {
    if (!can_handler.initSocket()) {
        return -1;
    }

    ccan_handler.readActualPosition(TILT_CAN_ID);
    can_handler.readActualPosition(PAN_CAN_ID);
    usleep(500000);

    can_handler.writeOperationMode(TILT_CAN_ID, 3);
    can_handler.writeOperationMode(PAN_CAN_ID, 3);
    usleep(500000);

    can_handler.writeMotionMode(TILT_CAN_ID, (can_handler.getMode() == 0) ? 1 : 0);
    can_handler.writeMotionMode(PAN_CAN_ID, (can_handler.getMode() == 0) ? 1 : 0);
    usleep(500000);

    can_handler.writeProfileAcceleration(TILT_CAN_ID, 200000);
    can_handler.writeProfileAcceleration(PAN_CAN_ID, 200000);
    usleep(500000);

    can_handler.writeProfileDeceleration(TILT_CAN_ID, 200000);
    can_handler.writeProfileDeceleration(PAN_CAN_ID, 200000);
    usleep(500000);

    can_handler.writeProfileSpeed(TILT_CAN_ID, 150000);
    can_handler.writeProfileSpeed(PAN_CAN_ID, 150000);
    usleep(500000);

    can_handler.enableMotor(TILT_CAN_ID, true);
    can_handler.enableMotor(PAN_CAN_ID, true);
    usleep(500000);

    can_handler.writeTargetAbsolutePosition(TILT_CAN_ID, can_handler.getCurrentPosition(TILT_CAN_ID));
    can_handler.writeTargetAbsolutePosition(PAN_CAN_ID, can_handler.getCurrentPosition(PAN_CAN_ID));
    usleep(500000);

    can_handler.startMotion(TILT_CAN_ID);
    can_handler.startMotion(PAN_CAN_ID);

    usleep(500000);

    can_handler.subscribeTopics();
    can_handler.process();

    _is_running = false;
    return 0;
}

static void usage(const char *reason) {
    if (reason) {
        PX4_ERR("%s", reason);
    }
    PX4_INFO("usage: stabilisation {start|stop|status} {position|velocity} [initial_angle]");
}

extern "C" __EXPORT int stabilisation_main(int argc, char *argv[]) {
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    can_handler.setArguments(argc, argv);

    // PX4_INFO("Stored arguments:");
    // for (int i = 0; i < can_handler.getArgCount(); ++i) {
    //     PX4_INFO("arguments[%d]: %s", i, can_handler.getArgument(i));
    // }

    if (!strcmp(argv[1], "s")) {
        if (argc < 3) {
            usage("missing mode for start command");
            return 1;
        }

        if (_is_running) {
            PX4_WARN("stabilisation already running");
            return 0;
        }
        if (!strcmp(argv[2], "p")) {
            can_handler.setMode(0);
        } else if (!strcmp(argv[2], "v")) {
            can_handler.setMode(1);
        } else {
            usage("unrecognized mode");
            return 1;
        }

        daemon_task = px4_task_spawn_cmd("stabilisation",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2000,
                                         (px4_main_t)stabilisation_thread_main,
                                         (char *const *)argv);

        CANHandler::thread_should_exit = false;
        if (daemon_task < 0) {
            PX4_ERR("Failed to start stabilisation task");
            return -1;
        }
        PX4_INFO("stabilisation started successfully");
        return 0;
    }

    if (!strcmp(argv[1], "stp")) {
        if (daemon_task < 0) {
            PX4_WARN("stabilisation not running");
            return 0;
        }
        CANHandler::thread_should_exit = true;
        PX4_INFO("thread_should_exit updated");
        can_handler.setRunLoop(false);
        while (_is_running) {
            usleep(100000);
        }
        PX4_INFO("Stopping CAN sender.");
        can_handler.setRunLoop(true);
        usleep(100000);

        if (can_handler.initSocket()) {
            can_handler.stopMotion(TILT_CAN_ID);
            usleep(500000);
            can_handler.stopMotion(PAN_CAN_ID);
            usleep(500000);

            can_handler.enableMotor(TILT_CAN_ID, false);
            usleep(500000);
            can_handler.enableMotor(PAN_CAN_ID, false);
            usleep(500000);

            can_handler.closeSocket();
        } else {
            PX4_ERR("Failed to reinitialize CAN socket for stopping commands");
        }

        PX4_INFO("stabilisation stopped");
        return 0;
    }

    if (!strcmp(argv[1], "sta")) {
        if (_is_running) {
            PX4_INFO("stabilisation is running");
        } else {
            PX4_INFO("stabilisation is not running");
        }
        return 0;
    }

    if (!strcmp(argv[1], "u")) {
        if (argc < 3) {
            usage("missing angle for update_angle command");
            return 1;
        }

        can_handler.setArguments(argc, argv);
        return 0;
    }

    usage("unrecognized command");
    return 1;
}
