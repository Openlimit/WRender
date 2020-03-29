#include <iostream>
#include <algorithm>
#include "camera.h"
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
	rotate_speed = 0.2;
	dnear = 1;
	dfar = 5;

	model = new Model("obj/diablo3_pose.obj", "obj/diablo3_pose_diffuse.tga",
		"obj/diablo3_pose_nm.tga", "obj/diablo3_pose_spec.tga");
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);

	shadowBuffer = new float[width * height];

	//SpotLight *light= new SpotLight();
	//light->position = Vec3f(1, 1, 2);
	//light->color = Vec3f(1, 1, 1);
	//light->front = -Vec3f(1, 1, 2);
	//light->front.normalize();
	//light->inner_cos = 0.9;
	//light->outter_cos = 0.8;

	PointLight* light = new PointLight();
	light->position = Vec3f(1, 1, 2);
	light->color = Vec3f(1, 1, 1);

	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	Camera camera_light(light->position, lookatpos, up);

	model_mat = Mat4f::Identity();
	Mat4f ligth_view_mat = camera_light.get_view();
	project_mat = perspective(60 * PI / 180, 1, dnear, dfar);
	light_MVP = project_mat * ligth_view_mat * model_mat;

	depthShader = new DepthShader();
	depthShader->model_mat = model_mat;
	depthShader->view_mat = ligth_view_mat;
	depthShader->project_mat = project_mat;

	cur_camera_pos = Vec3f(1, 0, 2);
	Camera camera(cur_camera_pos, lookatpos, up);
	view_mat = camera.get_view();

	//goochShader = new GoochShader();
	shader = new BlinnPhongShader();
	shader->light = dynamic_cast<Light*>(light);
	shader->viewPos = cur_camera_pos;
	shader->model_mat = model_mat;
	shader->view_mat = view_mat;
	shader->project_mat = project_mat;
	shader->model_mat_IT = model_mat.inverse().transpose();
	shader->width = width;
	shader->height = height;
	shader->shadow_M = light_MVP * (project_mat * view_mat * model_mat).inverse();
	shader->shadowBuffer = shadowBuffer;

	renderer->render(model, depthShader, nullptr);
	renderer->get_zbuffer(shadowBuffer);
}

void FrameRender::init_deffered(int width, int height) {
	frame_width = width;
	frame_height = height;
	rotate_speed = 0.2;
	dnear = 1;
	dfar = 5;

	model = new Model("obj/diablo3_pose.obj", "obj/diablo3_pose_diffuse.tga",
		"obj/diablo3_pose_nm.tga", "obj/diablo3_pose_spec.tga");
	/*model = new Model("obj/african_head.obj", "obj/african_head_diffuse.tga",
		"obj/african_head_nm.tga", "obj/african_head_spec.tga");*/
	deffered_model = new Model("obj/deffered_model.obj");
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);
	renderer->enable_deffered_rendering();

	PointLight* light1 = new PointLight();
	light1->position = Vec3f(1, 1, 2);
	light1->color = Vec3f(1, 1, 1);
	PointLight* light2 = new PointLight();
	light2->position = Vec3f(-1, 1, -2);
	light2->color = Vec3f(1, 1, 1);
	std::vector<Light*> lights;
	lights.emplace_back(light1);
	lights.emplace_back(light2);

	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	cur_camera_pos = Vec3f(1, 0, 2);
	Camera camera(cur_camera_pos, lookatpos, up);
	view_mat = camera.get_view();
	model_mat = Mat4f::Identity();
	project_mat = perspective(60 * PI / 180, 1, dnear, dfar);

	geoShader = new GeometryPassShader();
	geoShader->model_mat = model_mat;
	geoShader->view_mat = view_mat;
	geoShader->project_mat = project_mat;
	geoShader->model_mat_IT = model_mat.inverse().transpose();

	shadingShader = new ShadingPassShader();
	shadingShader->lights = lights;
	shadingShader->viewPos = cur_camera_pos;
}

void FrameRender::render() {
	renderer->clear_zbuffer();
	renderer->set_defferPass(Renderer::GEOMETRY);
	renderer->render(model, geoShader, nullptr);
	renderer->clear_zbuffer();
	renderer->set_defferPass(Renderer::SHADING);
	renderer->render(deffered_model, shadingShader, screenBits);
}

void FrameRender::release() {
	if (model != nullptr)
		delete model;
	if (renderer != nullptr)
		delete renderer;
	if (depthShader != nullptr)
		delete depthShader;
	if (shader != nullptr)
		delete shader;
	if (shadowBuffer != nullptr)
		delete shadowBuffer;
}

void FrameRender::resize(int width, int height) {
	if (renderer != nullptr)
		delete renderer;
	renderer = new Renderer(dnear, dfar, width, height, 0, 0, false);
	renderer->enable_deffered_rendering();

	/*if (shadowBuffer != nullptr)
		delete shadowBuffer;
	shadowBuffer = new float[width * height];

	shader->width = width;
	shader->height = height;
	shader->shadowBuffer = shadowBuffer;*/

	frame_width = width;
	frame_height = height;

	//renderer->render(model, depthShader, nullptr);
	//renderer->get_zbuffer(shadowBuffer);
}

void FrameRender::turn(float delta_x, float delta_y){
	float theta_x = -2 * PI * delta_x / frame_height;
	float theta_y = -2 * PI * delta_y / frame_height;

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

	cur_camera_pos = rotation_x * rotation_y * cur_camera_pos;
	update_camera();
}

void FrameRender::zoom(float delta) {
	float d = std::fmin(std::fmax(0.5, 1. - delta / 1200), 2);
	cur_camera_pos = cur_camera_pos * d;
	update_camera();
}

void FrameRender::update_camera() {
	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	Camera camera(cur_camera_pos, lookatpos, up);
	view_mat = camera.get_view();

	/*shader->viewPos = cur_camera_pos;
	shader->view_mat = view_mat;
	shader->shadow_M = light_MVP * (project_mat * view_mat * model_mat).inverse();*/

	geoShader->view_mat = view_mat;
	shadingShader->viewPos = cur_camera_pos;
}