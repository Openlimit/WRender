#pragma once
#include "shader.h"
#include "base_render.h"

class FrameRender:public BaseRender {
public:
	FrameRender() = default;

	virtual ~FrameRender() {
		release();
	}

	void init(int width, int height) override;

	void render() override;

	void release() override;

	void resize(int width, int height) override;
	
private:
	void update_view() override;

	void generate_ShadowMap();

	void init_SSAO();

	void init_skybox();

	float dnear;
	float dfar;

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

	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
};