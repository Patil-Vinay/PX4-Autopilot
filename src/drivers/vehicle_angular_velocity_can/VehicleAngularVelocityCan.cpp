#include "VehicleAngularVelocityCan.hpp"
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <nuttx/can/can.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <cstring>
#include <unistd.h>

VehicleAngularVelocityCan::VehicleAngularVelocityCan() :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
    _angular_velocity_sub(-1)
{
}

VehicleAngularVelocityCan::~VehicleAngularVelocityCan()
{
}

void VehicleAngularVelocityCan::Run()
{
    if (should_exit()) {
        exit_and_cleanup();
        return;
    }

    if (!_initialized) {
        _fd = ::open("/dev/can0", O_RDWR);
        if (_fd < 0) {
            PX4_INFO("FAILED TO OPEN /dev/can0");
            return;
        }
        _initialized = true;
    }

    if (_angular_velocity_sub == -1) {
        _angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    }

    send_vehicle_angular_velocity();
}

void VehicleAngularVelocityCan::send_vehicle_angular_velocity()
{
    struct vehicle_angular_velocity_s angular_velocity;
    if (orb_copy(ORB_ID(vehicle_angular_velocity), _angular_velocity_sub, &angular_velocity) == PX4_OK) {
        can_msg_s msg {};
        msg.cm_hdr.ch_id = 0x123; // Set your CAN ID here
        msg.cm_hdr.ch_dlc = sizeof(float); // Sending only one float (4 bytes)
        
        float z_angular_velocity = angular_velocity.xyz[2];
        std::memcpy(msg.cm_data, &z_angular_velocity, sizeof(z_angular_velocity));
        ::write(_fd, &msg, sizeof(msg));
    }
    ScheduleNow(); // Schedule the next run immediately
}

int VehicleAngularVelocityCan::start()
{
    // Schedule the work item to run immediately
    ScheduleNow();
    return PX4_OK;
}

int VehicleAngularVelocityCan::task_spawn(int argc, char *argv[])
{
    VehicleAngularVelocityCan *instance = new VehicleAngularVelocityCan();
    if (!instance) {
        PX4_ERR("driver allocation failed");
        return PX4_ERROR;
    }
    _object.store(instance);
    _task_id = task_id_is_work_queue;
    instance->start();
    return 0;
}

int VehicleAngularVelocityCan::print_usage(const char *reason)
{
    if (reason) {
        printf("%s\n\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Driver to send VehicleAngularVelocity Z component over CAN continuously.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("vehicle_angular_velocity_can", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int VehicleAngularVelocityCan::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_INFO("not running");
        return PX4_ERROR;
    }
    return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int vehicle_angular_velocity_can_main(int argc, char *argv[])
{
    return VehicleAngularVelocityCan::main(argc, argv);
}

