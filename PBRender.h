#pragma once
#include "base_render.h"
#include "shader.h"

class PBRender:public BaseRender
{
public:
	PBRender() = default;

	virtual ~PBRender() {
		release();
	}

	void init(int width, int height) override;

	void render() override;

	void release() override;

	void resize(int width, int height) override;

private:
	void update_view() override;

	void init_sphere();

	void init_enviroment_map();

	void init_precompute_map();

	Renderer* renderer;

	float dnear;
	float dfar;
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;

	Model* model;
	Model* skybox_model;

	PBRShader* shader;
	EnviromentBoxShader* skyboxShader;

	Texture3f* enviroment_map_hdr;
	TextureCube<Vec4f>* enviroment_map_cube;
	TextureCube<Vec4f>* irradiance_map_cube;
};

