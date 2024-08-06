#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/fog_data.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>

extern "C" __EXPORT int combined_reader_main(int argc, char *argv[]);

int combined_reader_main(int argc, char *argv[])
{
    PX4_INFO("Starting combined_reader");

    // Open a file for writing
    int fd = open("/fs/microsd/gyro_fog_log.csv", O_CREAT | O_WRONLY | O_TRUNC);
    if (fd < 0) {
        PX4_ERR("Failed to open file");
        return -1;
    }

    // Write the CSV header
    dprintf(fd, "Timestamp,Gyro_X,Gyro_Y,Gyro_Z,FOG_X,FOG_Y,FOG_Z\n");

    // Subscribe to sensor_gyro topic
    int sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
    if (sensor_gyro_sub < 0) {
        PX4_ERR("Failed to subscribe to sensor_gyro topic");
        close(fd);
        return -1;
    }

    // Subscribe to fog_data topic
    int fog_data_sub = orb_subscribe(ORB_ID(fog_data));
    if (fog_data_sub < 0) {
        PX4_ERR("Failed to subscribe to fog_data topic");
        close(fd);
        return -1;
    }

    struct sensor_gyro_s gyro_data;
    memset(&gyro_data, 0, sizeof(gyro_data));

    struct fog_data_s fog_data;
    memset(&fog_data, 0, sizeof(fog_data));

    const float rad_to_deg = 180.0f / (float)M_PI;

    while (true) {
        bool new_gyro_data = false;
        bool new_fog_data = false;

        // Check for new gyro data
        if (orb_copy(ORB_ID(sensor_gyro), sensor_gyro_sub, &gyro_data) == PX4_OK) {
            new_gyro_data = true;
        }

        // Check for new fog data
        if (orb_copy(ORB_ID(fog_data), fog_data_sub, &fog_data) == PX4_OK) {
            new_fog_data = true;
        }

        // Print and log both data in one line if both have new data
        if (new_gyro_data && new_fog_data) {
            // Convert gyro data to degrees per second
            float gyro_x_deg = gyro_data.x * rad_to_deg;
            float gyro_y_deg = gyro_data.y * rad_to_deg;
            float gyro_z_deg = gyro_data.z * rad_to_deg;

            PX4_INFO("Gyro Data: [%.4f, %.4f, %.4f], FOG Data: [%.4f, %.4f, %.4f]",
                      (double)gyro_x_deg, (double)gyro_y_deg, (double)gyro_z_deg,
                      (double)fog_data.x, (double)fog_data.y, (double)fog_data.z);

            // Write to the CSV log file
            dprintf(fd, "%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    hrt_absolute_time(),
                    (double)gyro_x_deg, (double)gyro_y_deg, (double)gyro_z_deg,
                    (double)fog_data.x, (double)fog_data.y, (double)fog_data.z);
        }

        // Sleep for 200 milliseconds
        usleep(100000);
    }

    // Close the file (note: this will never be reached in this infinite loop)
    close(fd);
    orb_unsubscribe(sensor_gyro_sub);
    orb_unsubscribe(fog_data_sub);
    return 0;
}
