#include <math.h>
#include <Arduino.h>

bool isApproxEqual(const float ax, const float ay, const float az, const float bx, const float by, const float bz, const float tiltTol);
uint64_t speakMatrix(uint64_t input, uint8_t columns, bool inverted, long colorArray[8][8] = nullptr);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);