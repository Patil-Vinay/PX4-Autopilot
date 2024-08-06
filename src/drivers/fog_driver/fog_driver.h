#pragma once

#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
// #include <uORB/topics/fog_data.h>

struct FOG_DATA_Type {
    int32_t x;
    int32_t y;
    int32_t z;
};

class FOGDriver : public ModuleBase<FOGDriver>
{
public:
    FOGDriver();
    virtual ~FOGDriver();
    void run() override;

    static int task_spawn(int argc, char *argv[]); // Add this line
    static FOGDriver *instantiate(int argc, char *argv[]); // Add this line
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]); // Add this line

private:
    void readData();
    int uart_fd;
    orb_advert_t _gyro_pub;
};
