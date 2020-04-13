#pragma once
#include "camera.h"

class BaseRender {
public:
	virtual void init(int width, int height) = 0;

	virtual void render() = 0;

	virtual void release() = 0;

	virtual void resize(int width, int height) = 0;

	virtual void update_view() {}

	virtual void turn(float delta_x, float delta_y) {
		float theta_x = -rotate_speed * PI * delta_x / frame_height;
		float theta_y = -rotate_speed * PI * delta_y / frame_height;
		camera->turn(theta_x, theta_y);
		update_view();
	}

	virtual void zoom(float delta) {
		float d = std::fmin(std::fmax(0.5, 1 - delta / 1200), 2);
		camera->zoom(d);
		update_view();
	}

	unsigned char* screenBits;
	int frame_width;
	int frame_height;
	float rotate_speed;
	Camera* camera;
};
