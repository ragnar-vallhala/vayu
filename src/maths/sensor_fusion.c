#include "maths/sensor_fusion.h"
#include "maths/maths_interface.h"
#include <stddef.h> // For NULL
void sf_acc_mag(const float ax, const float ay, const float az,
                const float mx, const float my, const float mz,
                attitude_t *ori)
{
    if (ori == NULL)
    {
        return; // Handle null pointer
    }
    // Normalize accelerometer vector
    float norm_a = sf_sqrt(ax * ax + ay * ay + az * az);
    if (norm_a == 0.0f)
    {
        return; // Prevent division by zero
    }
    float ax_n = ax / norm_a;
    float ay_n = ay / norm_a;
    float az_n = az / norm_a;
    // Normalize magnetometer vector
    float norm_m = sf_sqrt(mx * mx + my * my + mz * mz);
    if (norm_m == 0.0f)
    {
        return; // Prevent division by zero
    }
    float mx_n = mx / norm_m;
    float my_n = my / norm_m;
    float mz_n = mz / norm_m;
    // Calculate roll and pitch from accelerometer
    ori->roll = to_degrees(sf_atan2(ay_n, az_n));
    ori->pitch = to_degrees(sf_atan2(-ax_n, sf_sqrt(ay_n * ay_n + az_n * az_n)));
    // Calculate yaw from magnetometer
    float sin_roll = sf_sin(ori->roll);
    float cos_roll = sf_cos(ori->roll);
    float sin_pitch = sf_sin(ori->pitch);
    float cos_pitch = sf_cos(ori->pitch);
    float mx2 = mx_n * cos_pitch + mz_n * sin_pitch;
    float my2 = mx_n * sin_roll * sin_pitch + my_n * cos_roll - mz_n * sin_roll * cos_pitch;
    ori->yaw = to_degrees(sf_atan2(-my2, mx2));
}
