#pragma once
#include "shader.h"


class FrameRender {
public:
	unsigned char* screenBits;

	virtual ~FrameRender() {
		release();
	}

	void init(int width, int height);

	void render();

	void release();

	void resize(int width, int height);

	void turn_left();

	void turn_right();

	void turn_up();

	void turn_down();
	
private:
	void update_camera_pos();

	int frame_width;
	int frame_height;

	float rotate_speed;

	Model* model;
	Renderer* renderer;
	DepthShader* depthShader;
	ShadowShader* shadowShader;
	float *shadowBuffer;

	Vec3f cur_camera_pos;
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Mat4f light_MVP;
};