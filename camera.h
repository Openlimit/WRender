#pragma once
#include "geometry.h"

class Camera
{
public:
	Camera(Vec3f _pos, Vec3f _lookatpos, Vec3f _up);
	virtual ~Camera() = default;

	Mat4f get_view();

private:
	Vec3f pos;
	Vec3f front;
	Vec3f right;
	Vec3f up;
};

