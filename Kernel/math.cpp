#include "math.h"
#include <cmath>

float kernel::alg::radians(float x, float y)
{
    float rad = std::atan2(y, x);
    if (rad < 0) {
		rad += float(2 * M_PI);
    }
    return rad;
}

float kernel::alg::degrees(float x, float y)
{
    float rad = radians(x, y);
	return float(rad * 180.0f / M_PI);
}
