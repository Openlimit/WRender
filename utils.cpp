#include "utils.h"
#include <cmath>

float clamp01(float v) {
	return std::fmax(std::fmin(v, 1), 0);
}

Vec3f clamp1(Vec3f v) {
	Vec3f r;
	for (int i = 0; i < 3; i++)
	{
		r[i] = std::fmin(v[i], 1);
	}
	return r;
}

float lerp(float v1, float v2, float t) {
	return v1 + (v2 - v1) * t;
}