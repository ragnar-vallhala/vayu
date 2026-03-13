#ifndef MATHS_SENSOR_FUSION_H
#define MATHS_SENSOR_FUSION_H
#define PI 3.14159265358979323846f

float m_sin(float x);
float m_cos(float x);
float m_asin(float x);
float m_atan2(float y, float x);
float m_sqrt(float x);
float m_pow(float base, float exp);
#define to_radians(degrees) (degrees * (PI / 180.0f))
#define to_degrees(radians) (radians * (180.0f / PI))
typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

typedef struct {
  float *values;
  int length;
} vector_t;

typedef struct {
  float **data;
  int rows;
  int cols;
} matrix_t;

void normalize_vector(vector_t *v);
void normalize_quaternion(quaternion_t *q);
void quaternion_multiply(const quaternion_t *qa, const quaternion_t *qb,
                         quaternion_t *out);

#endif // !MATHS_SENSOR_FUSION_H
