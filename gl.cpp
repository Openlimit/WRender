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

Renderer::Renderer(int _width, int _height, int _viewport_x, int _viewport_y, int _depth, bool _use_msaa) :
	screen_width(_width), screen_height(_height)
{
	if (_use_msaa) {
		msaa_factor = MSAA_FACTOR;
		sample_num = _width * _height * msaa_factor;
	}
	else {
		msaa_factor = 1;
		sample_num = _width * _height;
	}

	zbuffer = new float[sample_num];
	for (int i = 0; i < sample_num; i++)
	{
		zbuffer[i] = FLT_MAX;
	}

	color_buffer = new unsigned char[sample_num * 4];
	
	viewport_mat = viewport(_viewport_x, _viewport_y, _width, _height, _depth);
}

bool Renderer::render(Model* model, IShader* shader, TGAImage& image, bool is_perspective)
{
	if (render(model, shader, is_perspective)) {
		for (int i = 0; i < screen_height; i++)
		{
			for (int j = 0; j < screen_width; j++)
			{
				int idx = i * screen_width + j;
				TGAColor color = resolve(idx);
				image.set(j, i, color);
			}
		}
		return true;
	}
	else {
		return false;
	}
}

bool Renderer::render(Model* model, IShader* shader, unsigned char* image, bool is_perspective)
{
	if (render(model, shader, is_perspective)) {
		if (image != nullptr) {
			for (int i = 0; i < screen_height; i++)
			{
				for (int j = 0; j < screen_width; j++)
				{
					int idx = i * screen_width + j;
					TGAColor color = resolve(idx);
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

TGAColor Renderer::resolve(int idx) {
	int start_id = idx * msaa_factor * 4;
	Vec4i color(0, 0, 0, 0);
	for (int i = 0; i < msaa_factor; i++)
	{
		int color_id = start_id + i * 4;
		for (int j = 0; j < 4; j++)
		{
			color[j] += color_buffer[color_id + j];
		}
	}
	float w = 1.0 / msaa_factor;
	return TGAColor(color[0] * w, color[1] * w, color[2] * w, color[3] * w);
}

bool Renderer::render(Model* model, IShader* shader, bool is_perspective) {
	memset(color_buffer, 0, sizeof(unsigned char) * sample_num * 4);
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
		triangle(screen_coords, shader);
	}
	return true;
}

void Renderer::triangle(Vec3f* pts, IShader* shader) {
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
			if (msaa_factor > 1) {
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
					continue;

				Vec3f pixel(p[0] + 0.5, p[1] + 0.5, 0);
				Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], pixel);
				Vec4f color;
				if (!shader->fragment(uv, color))
				{
					int x = p[0];
					int y = p[1];
					int idx = y * width + x;
					for (int s = 0; s < msaa_factor; s++)
					{
						int z_idx = idx * msaa_factor + s;
						if (cover[s] && zbuffer[z_idx] > sample_z[s])
						{
							zbuffer[z_idx] = sample_z[s];
							int color_idx = idx * msaa_factor * 4 + s * 4;
							for (int i = 0; i < 4; i++)
							{
								color_buffer[color_idx + i] = color[i] * 255;
							}
						}
					}
				}
			}
			else {
				Vec3f pixel(p[0] + 0.5, p[1] + 0.5, 0);
				Vec3f uv = barycentric_gl(pts[0], pts[1], pts[2], pixel);
				if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
					continue;

				Vec3f z_val(pts[0][2], pts[1][2], pts[2][2]);
				float z = shader->interpolation(uv, z_val);

				Vec4f color;
				if (!shader->fragment(uv, color))
				{
					int x = p[0];
					int y = p[1];
					int idx = y * width + x;
					if (zbuffer[idx] > z)
					{
						zbuffer[idx] = z;
						for (int i = 0; i < 4; i++)
						{
							color_buffer[idx * 4 + i] = color[i] * 255;
						}
					}
				}
			}
		}
	}
}