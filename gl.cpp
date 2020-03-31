#include "gl.h"
#define MSAA_FACTOR 4

Vec3f barycentric_gl(Vec3f t0, Vec3f t1, Vec3f t2, Vec3f p) {
	Vec3f AB = t1 - t0;
	Vec3f AC = t2 - t0;
	Vec3f PA = t0 - p;
	Vec3f v1(AB[0], AC[0], PA[0]);
	Vec3f v2(AB[1], AC[1], PA[1]);
	Vec3f uv = v1.cross(v2);
	if (std::abs(uv[2]) < 1e-4)return Vec3f(-1, 1, 1);
	return Vec3f(1.0 - (uv[0] + uv[1]) / uv[2], uv[0] / uv[2], uv[1] / uv[2]);
}

Vec3f msaa_sample(Vec3f pixel, int s) {
	int x = s % 2;
	int y = s / 2;
	Vec3f delta(0.25 + x * 0.5, 0.25 + y * 0.5, 0);
	return pixel + delta;
}

Renderer::Renderer(float _near, float _far, int _width, int _height, int _viewport_x, int _viewport_y, bool _use_msaa) :
	dnear(_near), dfar(_far), screen_width(_width), screen_height(_height), z_test(true), z_write(true), 
	culling_face(true), cullingMode(BACK), MRT(false)
{
	if (_use_msaa) {
		msaa_factor = MSAA_FACTOR;
	}
	else {
		msaa_factor = 1;
	}

	default_Buffer = new FrameBuffer(_width, _height, msaa_factor);
	default_Buffer->depth_buffer->clear(FLT_MAX);
	
	viewport_mat = viewport(_viewport_x, _viewport_y, _width, _height);
}

bool Renderer::render(Model* model, IShader* shader, TGAImage& image)
{
	if (render(model, shader)) {
		for (int i = 0; i < screen_height; i++)
		{
			for (int j = 0; j < screen_width; j++)
			{
				TGAColor color = resolve(j, i);
				image.set(j, i, color);
			}
		}
		return true;
	}
	else {
		return false;
	}
}

bool Renderer::render(Model* model, IShader* shader, unsigned char* image)
{
	if (render(model, shader)) {
		if (image != nullptr) {
			for (int i = 0; i < screen_height; i++)
			{
				for (int j = 0; j < screen_width; j++)
				{
					TGAColor color = resolve(j, i);
					int image_idx = (screen_height - i - 1) * screen_width + j;
					image[image_idx * 3] = color.b;
					image[image_idx * 3 + 1] = color.g;
					image[image_idx * 3 + 2] = color.r;
				}
			}
		}
		return true;
	}
	else {
		return false;
	}
}

TGAColor Renderer::resolve(int x, int y) {
	Vec4i color(0, 0, 0, 0);//unsigned char可能溢出
	for (int i = 0; i < msaa_factor; i++)
	{
		Vec4u cur_color = default_Buffer->color_buffer->get(x, y, i);
		color += cur_color.cast<int>();
	}
	float w = 1.0 / msaa_factor;
	return TGAColor(color[0] * w, color[1] * w, color[2] * w, color[3] * w);
}

bool Renderer::render(Model* model, IShader* shader) {
	default_Buffer->color_buffer->clear();

	shader->model = model;
	if (MRT)
		shader->buffers = G_Buffer->other_buffers.data();

	for (int i = 0; i < model->nfaces(); i++)
	{
		Vec4f clipping_coords[3];
		for (int j = 0; j < 3; j++) {
			clipping_coords[j] = shader->vertex(i, j);
		}
		if (ClipCulling(clipping_coords))
			continue;

		Vec3f screen_coords[3];
		for (int j = 0; j < 3; j++)
		{
			shader->z_invs_[j] = 1. / clipping_coords[j][3];//根据透视投影矩阵的性质(相机朝向z轴正方向),W=Z0

			Vec4f ndc_coord = clipping_coords[j] / clipping_coords[j][3]; //透视除法

			shader->ndc_verts[j] = ndc_coord.head(3);
			Vec4f screen_coord = viewport_mat * ndc_coord;
			screen_coords[j] = Vec3f(int(screen_coord[0]), int(screen_coord[1]), screen_coord[2]);
		}
		triangle(screen_coords, shader);
	}
	return true;
}

bool Renderer::FaceCulling(Vec3f *verts) {
	if (culling_face) {
		Vec3f normal = cal_normal(verts);
		bool culling = (cullingMode == BACK && normal[2] > 0) || (cullingMode == FRONT && normal[2] < 0);
		return culling;
	}
	else {
		return false;
	}
}

bool Renderer::ClipCulling(Vec4f* verts) {
	if (verts[0][3] < dnear || verts[1][3] < dnear || verts[2][3] < dnear)
		return true;
	if (verts[0][3] > dfar || verts[1][3] > dfar || verts[2][3] > dfar)
		return true;

	for (int i = 0; i < 3; i++)
	{
		if (verts[0][i] > verts[0][3] && verts[1][i] > verts[1][3] && verts[2][i] > verts[2][3])
			return true;
		if (verts[0][i] < -verts[0][3] && verts[1][i] < -verts[1][3] && verts[2][i] < -verts[2][3])
			return true;
	}

	return false;
}

void Renderer::triangle(Vec3f* pts, IShader* shader) {
	if (FaceCulling(shader->ndc_verts)) {
		return;
	}

	Vec2f max(0, 0);
	Vec2f min(screen_width, screen_height);
	for (int i = 0; i < 3; i++)
	{
		if (pts[i][0] > max[0])max[0] = pts[i][0];
		if (pts[i][1] > max[1])max[1] = pts[i][1];
		if (pts[i][0] < min[0])min[0] = pts[i][0];
		if (pts[i][1] < min[1])min[1] = pts[i][1];
	}

	max[0] = std::fmin(max[0], screen_width - 1);
	max[1] = std::fmin(max[1], screen_height - 1);
	min[0] = std::fmax(min[0], 0);
	min[1] = std::fmax(min[1], 0);

	Vec3f p;
	for (p[0] = min[0]; p[0] <= max[0]; p[0]++)
	{
		for (p[1] = min[1]; p[1] <= max[1]; p[1]++)
		{
			shader->frag_idx = Vec2i(p[0], p[1]);
			if (msaa_factor > 1) {
				msaa_process(shader, pts, p);
			}
			else {
				process(shader, pts, p);
			}
		}
	}
}

void Renderer::process(IShader* shader, Vec3f* pts, Vec3f p) {
	Vec3f pixel(p[0] + 0.5, p[1] + 0.5, 0);
	Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], pixel);
	if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
		return;

	Vec3f z_val(pts[0][2], pts[1][2], pts[2][2]);
	float z = shader->interpolation(uv, z_val);

	Vec4f color;
	if (!shader->fragment(uv, color))
	{
		bool z_passing = true;
		if (z_test)
			z_passing = default_Buffer->depth_buffer->get(p[0], p[1]) > z;

		if (z_passing)
		{
			if (z_write)
				default_Buffer->depth_buffer->set(p[0], p[1], 0, z);

			Vec4u colori = (color * 255).cast<unsigned char>();
			default_Buffer->color_buffer->set(p[0], p[1], 0, colori);
			if (MRT)
				shader->MRT(uv);
		}
	}
}

void Renderer::msaa_process(IShader* shader, Vec3f* pts, Vec3f p) {
	bool cover[MSAA_FACTOR];
	float sample_z[MSAA_FACTOR];
	bool not_cover = true;
	for (int s = 0; s < msaa_factor; s++) {
		Vec3f sample = msaa_sample(p, s);
		Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], sample);
		if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0) {
			cover[s] = false;
			continue;
		}
		cover[s] = true;
		not_cover = false;
		Vec3f z_val(pts[0][2], pts[1][2], pts[2][2]);
		sample_z[s] = shader->interpolation(uv, z_val);
	}

	if (not_cover)
		return;

	Vec3f pixel(p[0] + 0.5, p[1] + 0.5, 0);
	Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], pixel);
	Vec4f color;
	if (!shader->fragment(uv, color))
	{
		for (int s = 0; s < msaa_factor; s++)
		{
			bool z_passing = true;
			if (z_test)
				z_passing = default_Buffer->depth_buffer->get(p[0], p[1], s) > sample_z[s];

			if (cover[s] && z_passing)
			{
				if (z_write)
					default_Buffer->depth_buffer->set(p[0], p[1], s, sample_z[s]);

				Vec4u colori = (color * 255).cast<unsigned char>();
				default_Buffer->color_buffer->set(p[0], p[1], s, colori);
				if (MRT)
					shader->MRT(uv);
			}
		}
	}
}