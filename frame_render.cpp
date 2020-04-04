#include <iostream>
#include <algorithm>
#include <random>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "frame_render.h"

void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
	bool steep = false;
	if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}

	if (x0 > x1) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}

	for (int x = x0; x <=x1; x++){
		float t = (x - x0) / (float)(x1 - x0);
		int y = y0 + (y1 - y0) * t;
		if(steep)
			image.set(y, x, color);
		else
			image.set(x, y, color);
	}
}

void line(Vec2i v0, Vec2i v1, TGAImage& image, TGAColor color) {
	bool steep = false;
	if (std::abs(v0[0] - v1[0]) < std::abs(v0[1]-v1[1])) {
		std::swap(v0[0], v0[1]);
		std::swap(v1[0], v1[1]);
		steep = true;
	}

	if (v0[0] > v1[0]) {
		std::swap(v0[0], v1[0]);
		std::swap(v0[1], v1[1]);
	}

	for (int x = v0[0]; x <= v1[0]; x++) {
		float t = (x - v0[0]) / (float)(v1[0] - v0[0]);
		int y = (1 - t) * v0[1] + t * v1[1];
		if (steep)
			image.set(y, x, color);
		else
			image.set(x, y, color);
	}
}

void wireframe(const char* filename, TGAImage& image, TGAColor color) {
	Model model(filename);
	int width = image.get_width();
	int height = image.get_height();
	for (int i = 0; i < model.nfaces(); i++)
	{
		std::vector<Vec3i> indices = model.face(i);
		int s = indices.size();
		for (int j = 0; j < s; j++) {
			Vec3f v0 = model.vert(indices[j][0]);
			Vec3f v1 = model.vert(indices[(j + 1) % s][0]);
			int x0 = (v0[0] + 1.) * width / 2.;
			int y0 = (v0[1] + 1.) * height / 2.;
			int x1 = (v1[0] + 1.) * width / 2.;
			int y1 = (v1[1] + 1.) * height / 2.;
			line(x0, y0, x1, y1, image, color);
		}
	}
}

void triangle_sweepline(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage& image, TGAColor color) {
	if (t0[1] > t1[1]) { std::swap(t0[0], t1[0]); std::swap(t0[1], t1[1]); }
	if (t0[1] > t2[1]) { std::swap(t0[0], t2[0]); std::swap(t0[1], t2[1]); }
	if (t1[1] > t2[1]) { std::swap(t1[0], t2[0]); std::swap(t1[1], t2[1]); }
	for (int y = t0[1]; y <= t1[1]; y++)
	{
		float tA = (y - t0[1]) / (float)(t2[1] - t0[1]);
		float tB = (y - t0[1]) / (float)(t1[1] - t0[1]);
		int Ax = (1 - tA) * t0[0] + tA * t2[0];
		int Bx = (1 - tB) * t0[0] + tB * t1[0];
		if (Ax > Bx)std::swap(Ax, Bx);
		for (int x = Ax; x <= Bx; x++)
			image.set(x, y, color);
	}
	for (int y = t1[1]; y <= t2[1]; y++)
	{
		float tA = (y - t0[1]) / (float)(t2[1] - t0[1]);
		float tB = (y - t1[1]) / (float)(t2[1] - t1[1]);
		int Ax = (1 - tA) * t0[0] + tA * t2[0];
		int Bx = (1 - tB) * t1[0] + tB * t2[0];
		if (Ax > Bx)std::swap(Ax, Bx);
		for (int x = Ax; x <= Bx; x++)
			image.set(x, y, color);
	}
}

void FrameRender::init(int width, int height) {
	frame_width = width;
	frame_height = height;
	rotate_speed = 2;
	dnear = 0.5;
	dfar = 5;

	model_mat = Mat4f::Identity();
	project_mat = perspective(60 * PI / 180, 1, dnear, dfar);

	model = new Model("obj/diablo3_pose.obj", "obj/diablo3_pose_diffuse.tga",
		"obj/diablo3_pose_nm.tga", "obj/diablo3_pose_spec.tga");
	/*model = new Model("obj/african_head.obj", "obj/african_head_diffuse.tga",
		"obj/african_head_nm.tga", "obj/african_head_spec.tga");*/
	deffered_model = new Model("obj/deffered_model.obj");
	skybox_model = new Model("obj/skybox_model.obj");
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);
	renderer->enable_deffered_rendering();

	PointLight* light1 = new PointLight();
	light1->position = Vec3f(1, 1, 2);
	light1->color = Vec3f(1, 1, 1);
	PointLight* light2 = new PointLight();
	light2->position = Vec3f(-1, 1, -2);
	light2->color = Vec3f(1, 1, 1);
	lights.emplace_back(light1);
	lights.emplace_back(light2);

	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	for (int i = 0; i < lights.size(); i++)
	{
		Camera camera_light(lights[i]->position, lookatpos, up);
		Mat4f ligth_view = camera_light.get_view();
		Mat4f light_mat = project_mat * ligth_view;
		lightMats.emplace_back(light_mat);
		Texture1f *shadowMap = new Texture1f(width, height);
		shadowMaps.emplace_back(shadowMap);
	}
	
	Vec3f camera_pos(0, 0, 2);
	camera = new Camera(camera_pos, lookatpos, up);
	view_mat = camera->get_view();

	geoShader = new GeometryPassShader();
	geoShader->model_mat = model_mat;
	geoShader->view_mat = view_mat;
	geoShader->project_mat = project_mat;
	geoShader->model_mat_IT = model_mat.inverse().transpose();

	/*shadingShader = new ShadingPassShader();
	shadingShader->lights = lights;
	shadingShader->viewPos = camera->get_camera_pos();
	shadingShader->lightMats = lightMats;
	shadingShader->shadowMaps = shadowMaps;*/

	shadingShader = new ReflectShadingPassShader();
	shadingShader->viewPos = camera->get_camera_pos();

	ssaoShader = new SSAOShader();
	ssaoShader->project_mat = project_mat;
	ssaoShader->view_mat = view_mat;
	ssaoShader->radius = 1;
	init_SSAO();

	depthShader = new DepthShader();
	generate_ShadowMap();

	init_skybox();
}

void FrameRender::generate_ShadowMap() {
	renderer->set_cullingMode(Renderer::FRONT);
	for (int i = 0; i < lights.size(); i++)
	{
		depthShader->MVP = lightMats[i] * model_mat;
		renderer->clear_zbuffer();
		renderer->render(model, depthShader, nullptr);
		renderer->get_zbuffer(shadowMaps[i]);
	}
	renderer->set_cullingMode(Renderer::BACK);
}

void FrameRender::init_SSAO() {
	std::uniform_real_distribution<float> randomFloats(0.0, 1.0);
	std::default_random_engine generator;
	std::vector<Vec3f> ssaoKernel;
	for (int i = 0; i < 64; ++i)
	{
		Vec3f sample(
			randomFloats(generator) * 2.0 - 1.0,
			randomFloats(generator) * 2.0 - 1.0,
			randomFloats(generator)
			);
		sample.normalize();
		sample *= randomFloats(generator);
		float scale = i / 64.0;
		scale = lerp(0.1f, 1.0f, scale * scale);
		sample *= scale;
		ssaoKernel.push_back(sample);
	}

	std::vector<Vec3f> ssaoNoise;
	for (int i = 0; i < 16; i++)
	{
		Vec3f noise(
			randomFloats(generator) * 2.0 - 1.0,
			randomFloats(generator) * 2.0 - 1.0,
			0.0f);
		ssaoNoise.push_back(noise);
	}

	ssaoShader->ssaoKernel = ssaoKernel;
	ssaoShader->ssaoNoise = ssaoNoise;
}

void FrameRender::init_skybox() {
	std::vector<std::string> faces
	{
		"right.jpg",
		"left.jpg",
		"top.jpg",
		"bottom.jpg",
		"front.jpg",
		"back.jpg"
	};
	std::string pre = "obj/skybox/";
	int width, height, nrChannels;
	for (int i = 0; i < 6; i++)
	{
		unsigned char* data = stbi_load((pre + faces[i]).c_str(), &width, &height, &nrChannels, 0);
		assert(data != nullptr && nrChannels == 3);
		if (skybox == nullptr) {
			skybox = new TextureCube<Vec3u>(width, height);
		}
		skybox->init_from_data((Vec3u*)data, i);
		stbi_image_free(data);
	}
	
	skyboxShader = new SkyBoxShader();
	skyboxShader->view_mat= Mat4f::Identity();
	skyboxShader->view_mat.topLeftCorner(3, 3) = view_mat.topLeftCorner(3, 3);
	skyboxShader->project_mat = perspective(90 * PI / 180, 1, dnear, dfar);;
	skyboxShader->skybox = skybox;

	shadingShader->skybox = skybox;
}

void FrameRender::render() {
	renderer->clear_deffered_rendering();
	renderer->clear_colorbuffer();
	renderer->clear_zbuffer();

	renderer->render(model, geoShader, nullptr);

	//renderer->clear_zbuffer();
	//renderer->render(deffered_model, ssaoShader, screenBits);

	renderer->clear_zbuffer();
	renderer->render(deffered_model, shadingShader, screenBits);

	renderer->render(skybox_model, skyboxShader, screenBits);
}

void FrameRender::release() {
	if (model != nullptr)
		delete model;
	if (deffered_model != nullptr)
		delete deffered_model;
	if (renderer != nullptr)
		delete renderer;
	if (depthShader != nullptr)
		delete depthShader;
	if (geoShader != nullptr)
		delete geoShader;
	if (shadingShader != nullptr)
		delete shadingShader;
	if (ssaoShader != nullptr)
		delete ssaoShader;
	for (int i = 0; i < lights.size(); i++)
	{
		if (lights[i] != nullptr)
			delete lights[i];
	}
}

void FrameRender::resize(int width, int height) {
	if (renderer != nullptr)
		delete renderer;
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);
	renderer->enable_deffered_rendering();

	frame_width = width;
	frame_height = height;

	for (int i = 0; i < lights.size(); i++)
	{
		if (shadowMaps[i] != nullptr)
			delete shadowMaps[i];
		shadowMaps[i] = new Texture1f(width, height);
	}
	generate_ShadowMap();

	//shadingShader->shadowMaps = shadowMaps;
}

void FrameRender::turn(float delta_x, float delta_y){
	float theta_x = -rotate_speed * PI * delta_x / frame_height;
	float theta_y = -rotate_speed * PI * delta_y / frame_height;
	camera->turn(theta_x, theta_y);
	update_view();
}

void FrameRender::zoom(float delta) {
	float d = std::fmin(std::fmax(0.5, 1 - delta / 1200), 2);
	camera->zoom(d);
	update_view();
}

void FrameRender::update_view() {
	view_mat = camera->get_view();

	geoShader->view_mat = view_mat;
	shadingShader->viewPos = camera->get_camera_pos();
	ssaoShader->view_mat = view_mat;
	skyboxShader->view_mat.topLeftCorner(3, 3) = view_mat.topLeftCorner(3, 3);
}