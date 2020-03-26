#include "utils.h"
#include <cmath>

float clamp01(float v) {
	return std::fmax(std::fmin(v, 1), 0);
}