#pragma once
#include "wr_math.h"

float clamp01(float v);

Vec3f clamp1(Vec3f v);

float clamp(float x, float lowerlimit, float upperlimit);

float lerp(float v1, float v2, float t);

Vec4f lerp(Vec4f v1, Vec4f v2, float t);

Vec3f lerp(Vec3f v1, Vec3f v2, float t);

float smoothstep(float edge0, float edge1, float x);



