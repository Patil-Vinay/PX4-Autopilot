#include "HelloWorldCan.hpp"
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <nuttx/can/can.h>
#include <cstring>
#include <unistd.h>
#include <inttypes.h>  // Include for PRIu32

HelloWorldCan::HelloWorldCan() :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
    PX4_INFO("HelloWorldCan constructor called");
}

HelloWorldCan::~HelloWorldCan()
{
    if (_fd >= 0) {
        ::close(_fd);
        PX4_INFO("Closed CAN file descriptor");
    }
}

void HelloWorldCan::Run()
{
    PX4_INFO("Run method called");

    if (should_exit()) {
        PX4_INFO("Exiting and cleaning up");
        exit_and_cleanup();
        return;
    }

    if (!_initialized) {
        _fd = ::open("can0", O_RDWR);
        if (_fd < 0) {
            PX4_ERR("FAILED TO OPEN can0");
            return;
        }
        _initialized = true;
        PX4_INFO("CAN interface initialized");
    }

    send_hello_world();
}

void HelloWorldCan::send_hello_world()
{
    PX4_INFO("Preparing to send CAN message");

    struct can_msg_s msg {};
    msg.cm_hdr.ch_id = 0x456; // Set your CAN ID here
    msg.cm_hdr.ch_dlc = 8;
    msg.cm_data[0] = 0xAA;
    msg.cm_data[1] = 0xBB;
    msg.cm_data[2] = 0xCC;
    msg.cm_data[3] = 0xDD;
    msg.cm_data[4] = 0xEE;
    msg.cm_data[5] = 0xFF;
    msg.cm_data[6] = 0x11;
    msg.cm_data[7] = 0x22;

    PX4_INFO("CAN message prepared with ID: 0x%" PRIx32, msg.cm_hdr.ch_id);

    ssize_t nbytes = ::write(_fd, &msg, sizeof(msg));
    if (nbytes < 0) {
        PX4_ERR("Failed to send CAN message");
        return;
    }

    PX4_INFO("Sent CAN message with ID: 0x%" PRIx32, msg.cm_hdr.ch_id);

    helloworld_can_s helloworld_can_msg{};
    helloworld_can_msg.timestamp = hrt_absolute_time();
    // Populate the helloworld_can_msg with your data if needed.
    _helloworld_can_pub.publish(helloworld_can_msg);

    PX4_INFO("Published CAN message to uORB");
}

int HelloWorldCan::start()
{
    PX4_INFO("Start method called");

    // Schedule the work item to run every second (1s = 1e6 microseconds)
    ScheduleOnInterval(1000000); // Use 1e6 microseconds directly
    Run();
    return PX4_OK;
}

int HelloWorldCan::task_spawn(int argc, char *argv[])
{
    PX4_INFO("task_spawn method called");

    HelloWorldCan *instance = new HelloWorldCan();
    if (!instance) {
        PX4_ERR("driver allocation failed");
        return PX4_ERROR;
    }
    _object.store(instance);
    _task_id = task_id_is_work_queue;
    instance->start();
    return 0;
}

int HelloWorldCan::print_usage(const char *reason)
{
    if (reason) {
        printf("%s\n\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Driver to send "Hello World" over CAN every second.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("helloworld_can", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int HelloWorldCan::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_INFO("not running");
        return PX4_ERROR;
    }
    return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int helloworld_can_main(int argc, char *argv[])
{
    return HelloWorldCan::main(argc, argv);
}
