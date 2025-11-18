#ifndef VAYU_MATHS_SENSOR_FUSION_H
#define VAYU_MATHS_SENSOR_FUSION_H
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;
void sf_acc_mag(const float ax, const float ay, const float az,
                const float mx, const float my, const float mz,
                attitude_t *ori);

#endif // VAYU_MATHS_SENSOR_FUSION_H