#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>

extern "C" __EXPORT int gyro_printer_main(int argc, char *argv[]);

// Function to convert radians to degrees
float rad_to_deg(float rad) {
    return rad * 180.0f / (float)M_PI;
}

int gyro_printer_main(int argc, char *argv[])
{
    PX4_INFO("Starting gyro_printer");

    // Subscribe to the vehicle_attitude topic
    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    if (vehicle_attitude_sub < 0) {
        PX4_ERR("Failed to subscribe to vehicle_attitude topic");
        return -1;
    }

    struct vehicle_attitude_s attitude_data;
    memset(&attitude_data, 0, sizeof(attitude_data));

    while (true) {
        bool updated;
        orb_check(vehicle_attitude_sub, &updated);
        if (updated) {
            // Copy the new data into the attitude_data structure
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude_data);

            // Convert quaternion to Euler angles
            matrix::Quatf q(attitude_data.q);
            matrix::Eulerf euler(q);

            PX4_INFO("Attitude Data: Roll: %.1f deg, Pitch: %.1f deg, Yaw: %.1f deg",
                     (double)rad_to_deg(euler(0)),
                     (double)rad_to_deg(euler(1)),
                     (double)rad_to_deg(euler(2)));
        }

        // px4_usleep(100000); // Sleep for 100ms
    }

    orb_unsubscribe(vehicle_attitude_sub);
    return 0;
}
