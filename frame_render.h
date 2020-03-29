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

	void turn(float delta_x, float delta_y);

	void zoom(float delta);
	
private:
	void update_camera();

	void generate_ShadowMap();

	int frame_width;
	int frame_height;
	float dnear;
	float dfar;
	float rotate_speed;

	Model* model;
	Model* deffered_model;
	Renderer* renderer;
	DepthShader* depthShader;
	//GoochShader* goochShader;
	//BlinnPhongShader* shader;
	GeometryPassShader* geoShader;
	ShadingPassShader* shadingShader;

	std::vector<Light*> lights;
	std::vector<Mat4f> lightMats;
	std::vector<Texture*> shadowMaps;

	Vec3f cur_camera_pos;
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
};