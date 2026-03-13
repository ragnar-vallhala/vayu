#ifndef VAYU_MATHS_SENSOR_FUSION_H
#define VAYU_MATHS_SENSOR_FUSION_H
typedef struct {
  float roll;
  float pitch;
  float yaw;
} attitude_t;
void m_acc_mag(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, attitude_t *ori);

void m_complementary_filter(const float ax, const float ay, const float az,
                            const float gx, const float gy, const float gz,
                            const float mx, const float my, const float mz,
                            float dt, attitude_t *ori);

#endif // VAYU_MATHS_SENSOR_FUSION_H