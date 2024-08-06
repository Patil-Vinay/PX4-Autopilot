// WorkItemFOG.hpp

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gyro.h>

#include <fcntl.h>
#include <termios.h>
#include <cmath>

#define DEG_TO_RAD ((float)M_PI / 180.0f)
#define DEVICE_ID uint32_t(4325386)

class WorkItemFOG : public ModuleBase<WorkItemFOG>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    WorkItemFOG();
    ~WorkItemFOG() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    void init();

    int print_status() override;

private:
    void Run() override;
    int uart_fd{-1};
    uORB::Publication<sensor_gyro_s> _gyro_pub{ORB_ID(sensor_gyro)};

    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};


    struct FOG_DATA_Type {
        int32_t x;
        int32_t y;
        int32_t z;
    };
};
