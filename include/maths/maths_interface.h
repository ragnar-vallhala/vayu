#ifndef MATHS_SENSOR_FUSION_H
#define MATHS_SENSOR_FUSION_H
#define USE_STANDARD_MATH
#define PI 3.14159265358979323846f

float sf_sin(float x);
float sf_cos(float x);
float sf_atan2(float y, float x);
float sf_sqrt(float x);
float sf_pow(float base, float exp);
#define to_radians(degrees) (degrees * (PI / 180.0f))
#define to_degrees(radians) (radians * (180.0f / PI))
typedef struct
{
    float *values;
    int length;
} vector_t;

typedef struct
{
    float **data;
    int rows;
    int cols;
} matrix_t;

void normalize_vector(vector_t *v);
#endif // !MATHS_SENSOR_FUSION_H
