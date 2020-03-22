#include "camera.h"

Camera::Camera(Vec3f _pos, Vec3f _lookatpos, Vec3f _up)
{
	pos = _pos;
	front = _lookatpos - _pos;
	front.normalize();
	right = front.cross(_up);
	right.normalize();
	up = right.cross(front);
}

Mat4f Camera::get_view()
{
	Mat4f view;
	view << right[0], right[1], right[2], -right.dot(pos),
		up[0], up[1], up[2], -up.dot(pos),
		front[0], front[1], front[2], -front.dot(pos),
		0, 0, 0, 1;
	return view;
}
