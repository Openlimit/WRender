#include "geometry.h"
#include <iostream>

Mat4f orthographic(float left, float right, float bottom, float top, float near, float far)
{
	Mat4f scale;
	scale << 2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, 2 / (far - near), 0,
		0, 0, 0, 1;
	Mat4f translate;
	translate << 1, 0, 0, -(right + left) / 2,
		0, 1, 0, -(top + bottom) / 2,
		0, 0, 1, -(far + near) / 2,
		0, 0, 0, 1;
	Mat4f project = scale * translate;
	return project;
}

Mat4f perspective(float fov, float wh_ratio, float near, float far) {
	float half_height = std::tanf(fov / 2) * near;
	float half_width = half_height * wh_ratio;
	Mat4f orth = orthographic(-half_width, half_width, -half_height, half_height, near, far);
	Mat4f pers;
	pers << near, 0, 0, 0,
		0, near, 0, 0,
		0, 0, near + far, -near * far,
		0, 0, 1, 0;
	pers = orth * pers;
	return pers;  
}

Mat4f viewport(int x, int y, int width, int height) {
	Mat4f m;
	m << width / 2.f, 0, 0, width / 2.f + x,
		0, height / 2.f, 0, height / 2.f + y,
		0, 0, 1 / 2.f, 1 / 2.f,
		0, 0, 0, 1;
	return m;
}

Vec3f cal_normal(Vec3f* pts) {
	Vec3f AB = pts[1] - pts[0];
	Vec3f AC = pts[2] - pts[0];
	Vec3f n = AC.cross(AB);
	n.normalize();
	return n;
}