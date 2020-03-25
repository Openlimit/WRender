#include "gl.h"

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

Renderer::Renderer(int _width, int _height, int _viewport_x, int _viewport_y, int _depth) :
	screen_width(_width), screen_height(_height)
{
	zbuffer = new float[_width * _height];
	for (int i = 0; i < _width * _height; i++)
	{
		zbuffer[i] = FLT_MAX;
	}

	viewport_mat = viewport(_viewport_x, _viewport_y, _width, _height, _depth);
}

bool Renderer::render(Model* model, IShader* shader, TGAImage& image, bool is_perspective)
{
	shader->model = model;
	shader->is_perspective = is_perspective;
	for (int i = 0; i < model->nfaces(); i++)
	{
		Vec3f screen_coords[3];
		for (int j = 0; j < 3; j++) {
			Vec4f clipping_coord = shader->vertex(i, j);
			if (is_perspective)
			{
				shader->z_[j] = clipping_coord[3];//根据透视投影矩阵的性质(相机朝向z轴正方向),W=Z0
			}
			clipping_coord = clipping_coord / clipping_coord[3];
			shader->clipping_verts[j] = clipping_coord.head(3);
			Vec4f screen_coord = viewport_mat * clipping_coord;
			screen_coords[j] = Vec3f(int(screen_coord[0]), int(screen_coord[1]), clipping_coord[2]);
		}
		triangle(screen_coords, shader, image);
	}
	return true;
}

bool Renderer::render(Model* model, IShader* shader, unsigned char* image, bool is_perspective)
{
	shader->model = model;
	shader->is_perspective = is_perspective;
	for (int i = 0; i < model->nfaces(); i++)
	{
		Vec3f screen_coords[3];
		for (int j = 0; j < 3; j++) {
			Vec4f clipping_coord = shader->vertex(i, j);
			if (is_perspective)
			{
				shader->z_[j] = clipping_coord[3];//根据透视投影矩阵的性质(相机朝向z轴正方向),W=Z0
			}
			clipping_coord = clipping_coord / clipping_coord[3];
			shader->clipping_verts[j] = clipping_coord.head(3);
			Vec4f screen_coord = viewport_mat * clipping_coord;
			screen_coords[j] = Vec3f(int(screen_coord[0]), int(screen_coord[1]), clipping_coord[2]);
		}
		triangle(screen_coords, shader, image);
	}
	return true;
}

void Renderer::triangle(Vec3f* pts, IShader* shader, TGAImage& image) {
	int width = screen_width;
	int height = screen_height;

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
			Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], p);
			if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
				continue;

			Vec3f z_val(pts[0][2], pts[1][2], pts[2][2]);
			p[2] = shader->interpolation(uv, z_val);

			TGAColor color;
			if (!shader->fragment(uv, color))
			{
				int x = p[0];
				int y = p[1];
				int idx = y * width + x;
				if (zbuffer[idx] > p[2])
				{
					zbuffer[idx] = p[2];
					image.set(x, y, color);
				}
			}
		}
	}
}

void Renderer::triangle(Vec3f* pts, IShader* shader, unsigned char* image) {
	int width = screen_width;
	int height = screen_height;

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
			Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], p);
			if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
				continue;

			Vec3f z_val(pts[0][2], pts[1][2], pts[2][2]);
			p[2] = shader->interpolation(uv, z_val);

			TGAColor color;
			if (!shader->fragment(uv, color))
			{
				int x = p[0];
				int y = p[1];
				int idx = y * width + x;
				if (zbuffer[idx] > p[2])
				{
					zbuffer[idx] = p[2];
					if (image != nullptr) {
						idx = (screen_height - y - 1) * width + x;
						image[idx * 3] = color.b;
						image[idx * 3 + 1] = color.g;
						image[idx * 3 + 2] = color.r;
					}
				}
			}
		}
	}
}