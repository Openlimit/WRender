#pragma once
#include "geometry.h"

class Camera
{
public:
	Camera(Vec3f _pos, Vec3f _lookatpos, Vec3f _up) :pos(_pos), lookatpos(_lookatpos), up(_up) {
		updateFrame();
	}

	virtual ~Camera() = default;

	void turn(float theta_x, float theta_y) {
		//if (std::abs(theta_x) > std::abs(theta_y)) {
		//	float cos_x = std::cos(theta_x / 2);
		//	float sin_x = std::sin(theta_x / 2);
		//	Eigen::Quaternionf rotation_x(cos_x, 0, sin_x, 0);
		//	pos = rotation_x * pos;
		//}
		//else {
		//	float cos_y = std::cos(theta_y / 2);
		//	float sin_y = std::sin(theta_y / 2);
		//	Eigen::Quaternionf rotation_y(cos_y, sin_y, 0, 0);
		//	pos = rotation_y * pos;
		//}

		float cos_theta_x = std::cos(theta_x);
		float sin_theta_x = std::sin(theta_x);
		float cos_theta_y = std::cos(theta_y);
		float sin_theta_y = std::sin(theta_y);

		Mat3f rotation_x, rotation_y;

		rotation_x << cos_theta_x, 0, sin_theta_x,
			0, 1, 0,
			-sin_theta_x, 0, cos_theta_x;

		rotation_y << 1, 0, 0,
			0, cos_theta_y, -sin_theta_y,
			0, sin_theta_y, cos_theta_y;

		Vec3f cur_pos = pos - lookatpos;
		cur_pos = rotation_x * rotation_y * pos;
		pos = cur_pos + lookatpos;

		updateFrame();
	}

	void zoom(float delta) {
		pos = lookatpos + (pos - lookatpos) * delta;
	}

	Mat4f get_view()
	{
		Mat4f view;
		view << right[0], right[1], right[2], -right.dot(pos),
			up[0], up[1], up[2], -up.dot(pos),
			front[0], front[1], front[2], -front.dot(pos),
			0, 0, 0, 1;
		return view;
	}

	Vec3f get_camera_pos() {
		return pos;
	}

private:
	void updateFrame() {
		front = lookatpos - pos;
		front.normalize();
		right = front.cross(up);
		right.normalize();
		up = right.cross(front);
	}

	Vec3f pos;
	Vec3f lookatpos;
	Vec3f front;
	Vec3f right;
	Vec3f up;
};
