#ifndef MATHS_INTERFACE_H
#define MATHS_INTERFACE_H
#define PI 3.14159265358979323846f

float m_sin(float x);
float m_cos(float x);
float m_asin(float x);
float m_atan2(float y, float x);
float m_sqrt(float x);
float m_pow(float base, float exp);
float m_clamp(float val, float min, float max);
float m_fabsf(float x);
/* Non-zero iff x is NaN. Lets consumers guard against NaN without pulling in
 * <math.h> themselves (this module is the single owner of libm). */
int m_isnan(float x);
/* Non-zero iff x is finite (not NaN or +/-inf). */
int m_isfinite(float x);
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
void quaternion_conjugate(const quaternion_t *q, quaternion_t *out);
void quaternion_from_euler(float roll, float pitch, float yaw, quaternion_t *q);

#endif // !MATHS_INTERFACE_H
