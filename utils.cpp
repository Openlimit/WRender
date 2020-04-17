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

Vec3f bilinear(Vec3f *data, int width, int height, float u, float v) {
	float x = clamp(u * width, 0, width - 1);
	float y = clamp(v * height, 0, height - 1);

	float x0 = std::floorf(x);
	float y0 = std::floorf(y);
	float x1 = clamp(std::ceilf(x), 0, width - 1);
	float y1 = clamp(std::ceilf(y), 0, height - 1);

	int idx00 = y0 * width + x0;
	int idx01 = y1 * width + x0;
	int idx10 = y0 * width + x1;
	int idx11 = y1 * width + x1;

	float dx = x - x0;
	float dy = y - y0;

	Vec3f d0 = (1 - dx) * data[idx00] + dx * data[idx10];
	Vec3f d1 = (1 - dx) * data[idx01] + dx * data[idx11];
	Vec3f d = (1 - dy) * d0 + dy * d1;

	return d;
}

Vec4f bilinear(Vec4f* data, int width, int height, float u, float v) {
	float x = clamp(u * width, 0, width - 1);
	float y = clamp(v * height, 0, height - 1);

	float x0 = std::floorf(x);
	float y0 = std::floorf(y);
	float x1 = clamp(std::ceilf(x), 0, width - 1);
	float y1 = clamp(std::ceilf(y), 0, height - 1);

	int idx00 = y0 * width + x0;
	int idx01 = y1 * width + x0;
	int idx10 = y0 * width + x1;
	int idx11 = y1 * width + x1;

	float dx = x - x0;
	float dy = y - y0;

	Vec4f d0 = (1 - dx) * data[idx00] + dx * data[idx10];
	Vec4f d1 = (1 - dx) * data[idx01] + dx * data[idx11];
	Vec4f d = (1 - dy) * d0 + dy * d1;

	return d;
}