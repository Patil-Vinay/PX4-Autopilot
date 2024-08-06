#pragma once

#include <nuttx/can/can.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/helloworld_can.h>

class HelloWorldCan : public ModuleBase<HelloWorldCan>, public px4::ScheduledWorkItem
{
public:
    HelloWorldCan();
    virtual ~HelloWorldCan();
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[]);
    int start();

private:
    void Run() override;
    void send_hello_world();
    int _fd{-1};
    bool _initialized{false};

    struct can_msg_s {
        struct can_hdr_s {
            uint32_t ch_id;
            uint8_t ch_rtr;
            uint8_t ch_dlc;
            uint8_t ch_extid;
            uint8_t ch_unused;
        } cm_hdr;
        uint8_t cm_data[8];
    };

    uORB::Publication<helloworld_can_s> _helloworld_can_pub{ORB_ID(helloworld_can)};
};
