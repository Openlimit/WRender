#pragma once
#include "shader.h"
#include "camera.h"


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
	void update_view();

	void generate_ShadowMap();

	void init_SSAO();

	void init_skybox();

	int frame_width;
	int frame_height;
	float dnear;
	float dfar;
	float rotate_speed;

	Model* model;
	Model* deffered_model;
	Model* skybox_model;

	Renderer* renderer;
	DepthShader* depthShader;
	GeometryPassShader* geoShader;
	ShadingPassShader* shadingShader;
	//ReflectShadingPassShader* shadingShader;
	SSAOShader* ssaoShader;
	SkyBoxShader* skyboxShader;

	std::vector<Light*> lights;
	std::vector<Mat4f> lightMats;
	std::vector<Texture1f*> shadowMaps;
	TextureCube<Vec3u>* skybox;

	Camera* camera;
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
};