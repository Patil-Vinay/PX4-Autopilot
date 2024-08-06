// CANSender.hpp
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <netpacket/can.h>

class CANSender : public ModuleBase<CANSender>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    CANSender();
    ~CANSender() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    void Run() override;

    bool initSocket();
    void closeSocket();
    void sendCANCommand(const struct can_frame &frame);
    void handleReceivedFrame(const struct can_frame &frame);
    void sendCANCommandAndWait(const struct can_frame &frame);
    void sendReadDataCommandAndWait(uint16_t driveID, uint16_t commandIndex, uint16_t subIndex);
    void sendWriteDataCommandAndWait(uint16_t driveID, uint16_t commandIndex, uint32_t value, uint16_t subIndex);
    void can_quaternion_to_euler(const matrix::Quatf &q, float &roll, float &pitch, float &yaw);

    uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub;
    uORB::Subscription _vehicle_angular_velocity_sub;
    uORB::SubscriptionInterval _parameter_update_sub;

    perf_counter_t _loop_perf;
    perf_counter_t _loop_interval_perf;

    int _socket;
    uint16_t _current_can_id;

    int _mode;
    int32_t _current_position;
    bool _expecting_response;
    struct can_frame _last_sent_command;

    static constexpr float TWO_PI = 6.283185307179586f;
    static constexpr int TIMEOUT_MS = 2000;

    static constexpr uint16_t CMD_IDX_TARGET_ABSOLUTE_POSITION = 0x86;
    static constexpr uint16_t CMD_IDX_PROFILE_SPEED = 0x8A;
    static constexpr uint16_t CMD_IDX_START_MOTION = 0x83;

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::COM_CAN_MODE>) _param_com_can_mode
    )
};
