#include <iostream>
#include <algorithm>
#include "camera.h"
#include "frame_render.h"

TGAColor red = TGAColor(255, 0, 0, 255);
TGAColor green = TGAColor(0, 255, 0, 255);
TGAColor white = TGAColor(255, 255, 255, 255);

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

Vec3f barycentric(Vec2i t0, Vec2i t1, Vec2i t2, Vec2i p) {
	Vec2i AB = t1 - t0;
	Vec2i AC = t2 - t0;
	Vec2i PA = t0 - p;
	Vec3i v1(AB[0], AC[0], PA[0]);
	Vec3i v2(AB[1], AC[1], PA[1]);
	Vec3i uv = v1.cross(v2);
	if (uv[2] == 0)return Vec3f(-1, 1, 1);
	return Vec3f(1.0 - (uv[0] + uv[1]) / (float)uv[2], uv[0] / (float)uv[2], uv[1] / (float)uv[2]);
}

Vec3f barycentric(Vec3f t0, Vec3f t1, Vec3f t2, Vec3f p) {
	Vec3f AB = t1 - t0;
	Vec3f AC = t2 - t0;
	Vec3f PA = t0 - p;
	Vec3f v1(AB[0], AC[0], PA[0]);
	Vec3f v2(AB[1], AC[1], PA[1]);
	Vec3f uv = v1.cross(v2);
	if (std::abs(uv[2]) < 1e-4)return Vec3f(-1, 1, 1);
	return Vec3f(1.0 - (uv[0] + uv[1]) / uv[2], uv[0] / uv[2], uv[1] / uv[2]);
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage& image, TGAColor color) {
	Vec2i max, min;
	max[0] = std::max(std::max(t0[0], t1[0]), t2[0]);
	max[1] = std::max(std::max(t0[1], t1[1]), t2[1]);
	min[0] = std::min(std::min(t0[0], t1[0]), t2[0]);
	min[1] = std::min(std::min(t0[1], t1[1]), t2[1]);

	int width = image.get_width();
	int height = image.get_height();
	max[0] = std::min(max[0], width);
	max[1] = std::min(max[1], height);
	min[0] = std::max(min[0], 0);
	min[1] = std::max(min[1], 0);

	for (int x = min[0]; x <= max[0]; x++)
	{
		for (int y = min[1]; y <= max[1]; y++)
		{
			Vec2i p(x, y);
			Vec3f uv = barycentric(t0, t1, t2, p);
			if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
				continue;
			image.set(x, y, color);
		}
	}
}

void triangle(Vec3f *pts, float *zbuffer, TGAImage& image, TGAColor color) {
	int width = image.get_width();
	int height = image.get_height();
	Vec2f max(0, 0);
	Vec2f min(width, height);
	for (int i = 0; i < 3; i++)
	{
		if (pts[i][0] > max[0])max[0] = pts[i][0];
		if (pts[i][1] > max[1])max[1] = pts[i][1];
		if (pts[i][0] < min[0])min[0] = pts[i][0];
		if (pts[i][1] < min[1])min[1] = pts[i][1];
	}
	
	max[0] = std::fmin(max[0], width-1);
	max[1] = std::fmin(max[1], height-1);
	min[0] = std::fmax(min[0], 0);
	min[1] = std::fmax(min[1], 0);

	Vec3f p;
	for (p[0] = min[0]; p[0] <= max[0]; p[0]++)
	{
		for (p[1] = min[1]; p[1] <= max[1]; p[1]++)
		{
			Vec3f uv = barycentric(pts[0], pts[1], pts[2], p);
			if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
				continue;
			p[2] = uv[0] * pts[0][2] + uv[1] * pts[1][2] + uv[2] * pts[2][2];
			int x = p[0];
			int y = p[1];
			int id = y * width + x;
			if (zbuffer[id] < p[2])
			{
				zbuffer[id] = p[2];
				image.set(x, y, color);
			}
		}
	}
}

void triangle(Vec3f* pts, Vec3f* normals, Vec3f* texts, float* zbuffer, Vec3f light_dir, 
	TGAImage& texture_image, TGAImage& image) {
	int width = image.get_width();
	int height = image.get_height();
	int t_width = texture_image.get_width();
	int t_height = texture_image.get_height();
	Vec2f max(0, 0);
	Vec2f min(width, height);
	for (int i = 0; i < 3; i++)
	{
		if (pts[i][0] > max[0])max[0] = pts[i][0];
		if (pts[i][1] > max[1])max[1] = pts[i][1];
		if (pts[i][0] < min[0])min[0] = pts[i][0];
		if (pts[i][1] < min[1])min[1] = pts[i][1];
	}

	max[0] = std::fmin(max[0], width - 1);
	max[1] = std::fmin(max[1], height - 1);
	min[0] = std::fmax(min[0], 0);
	min[1] = std::fmax(min[1], 0);

	Vec3f p;
	for (p[0] = min[0]; p[0] <= max[0]; p[0]++)
	{
		for (p[1] = min[1]; p[1] <= max[1]; p[1]++)
		{
			Vec3f uv = barycentric(pts[0], pts[1], pts[2], p);
			if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
				continue;
			p[2] = uv[0] * pts[0][2] + uv[1] * pts[1][2] + uv[2] * pts[2][2];
			Vec3f normal = normals[0] * uv[0] + normals[1] * uv[1] + normals[2] * uv[2];
			normal.normalize();
			float d = std::fmax(0, normal.dot(light_dir));
			Vec3f text = texts[0] * uv[0] + texts[1] * uv[1] + texts[2] * uv[2];
			TGAColor color = texture_image.get(text[0] * t_width, text[1] * t_height);
			
			int x = p[0];
			int y = p[1];
			int id = y * width + x;
			if (zbuffer[id] > p[2])
			{
				zbuffer[id] = p[2];
				image.set(x, y, TGAColor(color.r * d, color.g * d, color.b * d, color.a));
			}
		}
	}
}

void render_obj(const char* filename, TGAImage& image, Vec3f light_dir) {
	Model model(filename);
	int width = image.get_width();
	int height = image.get_height();
	Vec2i screen_coords[3];
	Vec3f world_coords[3];
	for (int i = 0; i < model.nfaces(); i++)
	{
		std::vector<Vec3i> indices = model.face(i);
		for (int j = 0; j < 3; j++) {
			world_coords[j] = model.vert(indices[j][0]);
			screen_coords[j] = Vec2i((world_coords[j][0] + 1.) * width / 2., (world_coords[j][1] + 1.) * height / 2.);
		}
		Vec3f AB = world_coords[1] - world_coords[0];
		Vec3f AC = world_coords[2] - world_coords[0];
		Vec3f n = AC.cross(AB);
		n.normalize();

		float d = n.dot(light_dir);
		if (d < 0)continue;
		triangle(screen_coords[0], screen_coords[1], screen_coords[2], image,
			TGAColor(d * 255, d * 255, d * 255, 255));
	}
}

Vec3f world2screen(Vec3f v, int width, int height) {

	return Vec3f(int((v[0] + 1.) * width / 2.), int((v[1] + 1.) * height / 2.), v[2]);
}

void render(const char* filename, TGAImage& image, Vec3f light_dir) {
	Model model(filename);
	int width = image.get_width();
	int height = image.get_height();
	float* zbuffer = new float[width * height];
	for (int i = 0; i < width*height; i++)
	{
		zbuffer[i] = -std::numeric_limits<float>::max();
	}

	Vec3f screen_coords[3];
	Vec3f world_coords[3];
	for (int i = 0; i < model.nfaces(); i++)
	{
		std::vector<Vec3i> indices = model.face(i);
		for (int j = 0; j < 3; j++) {
			world_coords[j] = model.vert(indices[j][0]);
			screen_coords[j] = world2screen(world_coords[j], width, height);
		}
		Vec3f AB = world_coords[1] - world_coords[0];
		Vec3f AC = world_coords[2] - world_coords[0];
		Vec3f n = AC.cross(AB);
		n.normalize();
	
		float d = std::fmax(0, n.dot(light_dir));
		triangle(screen_coords, zbuffer, image, TGAColor(d * 255, d * 255, d * 255, 255));
	}
}

void render_texture(const char* filename, const char* texturename, TGAImage& image, Vec3f light_dir) {
	Model model(filename);
	TGAImage texture_image;
	texture_image.read_tga_file(texturename);
	texture_image.flip_vertically();
	int width = image.get_width();
	int height = image.get_height();
	float* zbuffer = new float[width * height];
	for (int i = 0; i < width * height; i++)
	{
		zbuffer[i] = std::numeric_limits<float>::max();
	}

	Vec3f cam_pos(1, 0, 1);
	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	Camera camera(cam_pos, lookatpos, up);
	Mat4f view = camera.get_view();
	Mat4f project = perspective(90 * PI / 180, 1, 0.01, 4);
	Mat4f vp = viewport(0, 0, width, height, 255);
	Mat4f mvpv = vp * project * view;

	Vec3f screen_coords[3];
	Vec3f normals[3];
	Vec3f texts[3];
	for (int i = 0; i < model.nfaces(); i++)
	{
		std::vector<Vec3i> indices = model.face(i);
		for (int j = 0; j < 3; j++) {
			Vec3f world_coord = model.vert(indices[j][0]);
			Vec4f world_coord4(world_coord[0], world_coord[1], world_coord[2], 1);
			Vec4f screen_coord = mvpv * world_coord4;
			screen_coord = screen_coord / screen_coord[3];
			screen_coords[j] = Vec3f(int(screen_coord[0]), int(screen_coord[1]), screen_coord[2]);
			texts[j] = model.text(indices[j][1]);
			normals[j] = model.normal(indices[j][2]);
		}
		triangle(screen_coords, normals, texts, zbuffer, light_dir, texture_image, image);
	}
}

int main1() {
	TGAImage image(1024, 1024, TGAImage::RGB);

	//wireframe("obj/african_head.obj", image, white);

	/*Vec2i t0[3] = { Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80) };
	Vec2i t1[3] = { Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180) };
	Vec2i t2[3] = { Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180) };
	triangle(t0[0], t0[1], t0[2], image, red);
	triangle(t1[0], t1[1], t1[2], image, white);
	triangle(t2[0], t2[1], t2[2], image, green);*/

	Vec3f light_dir(2, 2, 2);
	light_dir.normalize();
	//render_obj("obj/african_head.obj", image, light_dir);
	//render("obj/african_head.obj", image, light_dir);
	//render_texture("obj/african_head.obj", "obj/african_head_diffuse.tga", image, light_dir);

	Vec3f cam_pos(1, 0, 2);
	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	Camera camera(cam_pos, lookatpos, up);

	Mat4f model_mat = Mat4f::Identity();
	Mat4f view_mat = camera.get_view();
	Mat4f project_mat = perspective(60 * PI / 180, 1, 0.01, 4);
	Mat4f model_mat_IT = model_mat.inverse().transpose();

	TextureShader * shader = new TextureShader();
	//GouraudShader* shader = new GouraudShader();
	shader->light_dir = light_dir;
	shader->MVP = project_mat * view_mat * model_mat;
	shader->M = shader->MVP.topLeftCorner(3, 3);
	shader->MIT = shader->M.inverse().transpose();
	/*shader->model_mat = model_mat;
	shader->view_mat = view_mat;
	shader->project_mat = project_mat;*/
	//shader->model_mat_IT = model_mat_IT;
	//shader->viewPos = cam_pos;

	Model* model = new Model("obj/african_head.obj", "obj/african_head_diffuse.tga",
		"obj/african_head_nm.tga", "obj/african_head_spec.tga");

	//Model* diablo_model = new Model("obj/diablo3_pose.obj");

	Renderer* renderer = new Renderer(1024, 1024);
	renderer->render(model, shader, image);
	
	image.flip_vertically();
	image.write_tga_file("output_test.tga");

	return 0;
}

int main2() {
	int width = 1024;
	int height = 1024;
	TGAImage depth_image(width, height, TGAImage::RGB);

	Vec3f light_pos(1, 1, 2);
	Vec3f light_dir = light_pos;
	light_dir.normalize();

	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	Camera camera_light(light_pos, lookatpos, up);

	Mat4f model_mat = Mat4f::Identity();
	Mat4f ligth_view_mat = camera_light.get_view();
	Mat4f project_mat = perspective(60 * PI / 180, 1, 1, 5);
	Mat4f cam_MVP = project_mat * ligth_view_mat * model_mat;
	
	DepthShader* shader = new DepthShader();
	shader->model_mat = model_mat;
	shader->view_mat = ligth_view_mat;
	shader->project_mat = project_mat;

	Model* model = new Model("obj/diablo3_pose.obj", "obj/diablo3_pose_diffuse.tga",
		"obj/diablo3_pose_nm.tga", "obj/diablo3_pose_spec.tga");
	/*Model* model = new Model("obj/african_head.obj", "obj/african_head_diffuse.tga",
		"obj/african_head_nm.tga", "obj/african_head_spec.tga");*/

	Renderer* renderer = new Renderer(width, height);
	renderer->render(model, shader, depth_image);

	depth_image.flip_vertically();
	depth_image.write_tga_file("output_depth.tga");

	float* shadowBuffer = new float[width * height];
	renderer->get_zbuffer(shadowBuffer);
	renderer->clear_zbuffer();

	Vec3f cam_pos(1, 0, 2);
	Camera camera(cam_pos, lookatpos, up);
	Mat4f view_mat = camera.get_view();
	
	ShadowShader* shadowShader = new ShadowShader();
	shadowShader->light_dir = light_dir;
	shadowShader->viewPos = cam_pos;
	shadowShader->model_mat = model_mat;
	shadowShader->view_mat = view_mat;
	shadowShader->project_mat = project_mat;
	shadowShader->model_mat_IT = model_mat.inverse().transpose();
	shadowShader->shadowBuffer = shadowBuffer;
	shadowShader->width = width;
	shadowShader->height = height;
	shadowShader->shadow_M = cam_MVP * (project_mat * view_mat * model_mat).inverse();

	TGAImage image(width, height, TGAImage::RGB);
	renderer->render(model, shadowShader, image);
	image.flip_vertically();
	image.write_tga_file("output_shadow.tga");

	return 0;

}

void FrameRender::init(int width, int height) {
	model = new Model("obj/diablo3_pose.obj", "obj/diablo3_pose_diffuse.tga",
		"obj/diablo3_pose_nm.tga", "obj/diablo3_pose_spec.tga");
	renderer = new Renderer(width, height, 0, 0, 255, false);
	shadowBuffer = new float[width * height];

	SpotLight *light= new SpotLight();
	light->position = Vec3f(1, 1, 2);
	light->color = Vec3f(1, 1, 1);
	light->front = -Vec3f(1, 1, 2);
	light->front.normalize();
	light->inner_cos = 0.9;
	light->outter_cos = 0.8;

	Vec3f lookatpos(0, 0, 0);
	Vec3f up(0, 1, 0);
	Camera camera_light(light->position, lookatpos, up);

	model_mat = Mat4f::Identity();
	Mat4f ligth_view_mat = camera_light.get_view();
	project_mat = perspective(60 * PI / 180, 1, 1, 5);
	light_MVP = project_mat * ligth_view_mat * model_mat;

	depthShader = new DepthShader();
	depthShader->model_mat = model_mat;
	depthShader->view_mat = ligth_view_mat;
	depthShader->project_mat = project_mat;

	cur_camera_pos = Vec3f(1, 0, 2);
	Camera camera(cur_camera_pos, lookatpos, up);
	view_mat = camera.get_view();

	//shadowShader = new ShadowShader();
	//shadowShader->light_dir = light_dir;
	//shadowShader->viewPos = cur_camera_pos;
	//shadowShader->model_mat = model_mat;
	//shadowShader->view_mat = view_mat;
	//shadowShader->project_mat = project_mat;
	//shadowShader->model_mat_IT = model_mat.inverse().transpose();
	//shadowShader->width = width;
	//shadowShader->height = height;
	//shadowShader->shadow_M = light_MVP * (project_mat * view_mat * model_mat).inverse();
	//shadowShader->shadowBuffer = shadowBuffer;
	goochShader = new GoochShader();
	goochShader->light = dynamic_cast<Light*>(light);
	goochShader->viewPos = cur_camera_pos;
	goochShader->model_mat = model_mat;
	goochShader->view_mat = view_mat;
	goochShader->project_mat = project_mat;
	goochShader->model_mat_IT = model_mat.inverse().transpose();
	goochShader->width = width;
	goochShader->height = height;
	goochShader->shadow_M = light_MVP * (project_mat * view_mat * model_mat).inverse();
	goochShader->shadowBuffer = shadowBuffer;

	frame_width = width;
	frame_height = height;
	rotate_speed = 0.2;
}

void FrameRender::render() {
	renderer->clear_zbuffer();
	renderer->render(model, depthShader, nullptr);
	renderer->get_zbuffer(shadowBuffer);
	renderer->clear_zbuffer();
	renderer->render(model, goochShader, screenBits);
}

void FrameRender::release() {
	if (model != nullptr)
		delete model;
	if (renderer != nullptr)
		delete renderer;
	if (depthShader != nullptr)
		delete depthShader;
	if (goochShader != nullptr)
		delete goochShader;
	if (shadowBuffer != nullptr)
		delete shadowBuffer;
}

void FrameRender::resize(int width, int height) {
	if (renderer != nullptr)
		delete renderer;
	renderer = new Renderer(width, height, 0, 0, 255, false);

	if (shadowBuffer != nullptr)
		delete shadowBuffer;
	shadowBuffer = new float[width * height];

	goochShader->width = width;
	goochShader->height = height;
	goochShader->shadowBuffer = shadowBuffer;

	frame_width = width;
	frame_height = height;
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

	goochShader->viewPos = cur_camera_pos;
	goochShader->view_mat = view_mat;
	goochShader->shadow_M = light_MVP * (project_mat * view_mat * model_mat).inverse();
}