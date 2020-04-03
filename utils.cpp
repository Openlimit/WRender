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

Vec4f lerp(Vec4f v1, Vec4f v2, float t) {
	return v1 + (v2 - v1) * t;
}

Vec3f lerp(Vec3f v1, Vec3f v2, float t) {
	return v1 + (v2 - v1) * t;
}

float clamp(float x, float lowerlimit, float upperlimit) {
	if (x < lowerlimit)
		x = lowerlimit;
	if (x > upperlimit)
		x = upperlimit;
	return x;
}

float smoothstep(float edge0, float edge1, float x) {
	// Scale, bias and saturate x to 0..1 range
	x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
	// Evaluate polynomial
	return x * x * (3 - 2 * x);
}
