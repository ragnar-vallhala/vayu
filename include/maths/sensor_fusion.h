#ifndef VAYU_MATHS_SENSOR_FUSION_H
#define VAYU_MATHS_SENSOR_FUSION_H
#include "maths/maths_interface.h"

typedef struct {
  float roll;
  float pitch;
  float yaw;
  quaternion_t q;
} attitude_t;

typedef enum {
  SF_COMPLEMENTARY,
  SF_MAHONY,
} sensor_fusion_filter_t;

void m_acc_mag(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, attitude_t *ori);

void m_complementary_filter(const float ax, const float ay, const float az,
                            const float gx, const float gy, const float gz,
                            const float mx, const float my, const float mz,
                            attitude_t *ori);

void m_mahony_filter(const float ax, const float ay, const float az,
                     const float gx, const float gy, const float gz,
                     const float mx, const float my, const float mz,
                     attitude_t *ori);

#endif // VAYU_MATHS_SENSOR_FUSION_H