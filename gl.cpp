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
	dnear(_near), dfar(_far), screen_width(_width), screen_height(_height), 
	early_z_test(true), z_test(true), z_write(true),
	culling_face(true), cullingMode(BACK), MRT(false), gammaCorrect(false), toneMapping(false)
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

	gamma_inv = 1 / 2.2;
}

bool Renderer::render(Model* model, IShader* shader, unsigned char* image)
{
	if (render(model, shader)) {
		if (image != nullptr) {
			post_process(image);
		}
		return true;
	}
	else {
		return false;
	}
}

bool Renderer::render(Model* model, IShader* shader) {
	shader->model = model;
	if (MRT)
		shader->buffers = G_Buffer->other_buffers.data();

	Point points[3];
	Point screen_point[3];
	for (int i = 0; i < model->nfaces(); i++)
	{
		for (int j = 0; j < 3; j++) {
			points[j] = shader->vertex(i, j);
		}
		if (ClipCulling(points))
			continue;

		std::vector<Point> clipping_verts = SutherlandHodgeman(points[0], points[1], points[2]);
		int n = clipping_verts.size() - 3 + 1;
		for (int j = 0; j < n; j++)
		{
			std::vector<int> point_ids = { 0,j + 1,j + 2 };
			for (int k = 0; k < 3; k++)
			{
				int idx = point_ids[k];
				Vec4f ndc_coord = clipping_verts[idx].clipping_pos / clipping_verts[idx].clipping_pos[3]; //透视除法,根据透视投影矩阵的性质(相机朝向z轴正方向),W=Z0
				Vec4f screen_coord = viewport_mat * ndc_coord;

				screen_point[k] = clipping_verts[idx];
				screen_point[k].ndc_pos = ndc_coord.head(3);
				screen_point[k].screen_pos = Vec3f(int(screen_coord[0]), int(screen_coord[1]), screen_coord[2]);
			}
			triangle(screen_point, shader);
		}
	}
	return true;
}

bool Renderer::FaceCulling(Point* verts) {
	if (culling_face) {
		Vec3f normal = cal_normal(verts[0].ndc_pos, verts[1].ndc_pos, verts[2].ndc_pos);
		bool culling = (cullingMode == BACK && normal[2] > 0) || (cullingMode == FRONT && normal[2] < 0);
		return culling;
	}
	else {
		return false;
	}
}

bool Renderer::ClipCulling(Point* verts) {
	if (verts[0].clipping_pos[3] < dnear && verts[1].clipping_pos[3] < dnear && verts[2].clipping_pos[3] < dnear)
		return true;
	if (verts[0].clipping_pos[3] > dfar && verts[1].clipping_pos[3] > dfar && verts[2].clipping_pos[3] > dfar)
		return true;

	for (int i = 0; i < 3; i++)
	{
		if (verts[0].clipping_pos[i] > verts[0].clipping_pos[3] && verts[1].clipping_pos[i] > verts[1].clipping_pos[3] && verts[2].clipping_pos[i] > verts[2].clipping_pos[3])
			return true;
		if (verts[0].clipping_pos[i] < -verts[0].clipping_pos[3] && verts[1].clipping_pos[i] < -verts[1].clipping_pos[3] && verts[2].clipping_pos[i] < -verts[2].clipping_pos[3])
			return true;
	}

	return false;
}

void Renderer::triangle(Point* pts, IShader* shader) {
	if (FaceCulling(pts)) {
		return;
	}

	Vec2f max(0, 0);
	Vec2f min(screen_width, screen_height);
	for (int i = 0; i < 3; i++)
	{
		if (pts[i].screen_pos[0] > max[0])max[0] = pts[i].screen_pos[0];
		if (pts[i].screen_pos[1] > max[1])max[1] = pts[i].screen_pos[1];
		if (pts[i].screen_pos[0] < min[0])min[0] = pts[i].screen_pos[0];
		if (pts[i].screen_pos[1] < min[1])min[1] = pts[i].screen_pos[1];
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
			if (msaa_factor > 1) {
				msaa_process(shader, pts, p);
			}
			else {
				process(shader, pts, p);
			}
		}
	}
}

void Renderer::process(IShader* shader, Point* pts, Vec3f p) {
	Vec3f pixel(p[0] + 0.5, p[1] + 0.5, 0);
	Vec3f uv = barycentric_gl(pts[0].screen_pos, pts[1].screen_pos, pts[2].screen_pos, pixel);
	if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
		return;

	Vec3f z_val(pts[0].screen_pos[2], pts[1].screen_pos[2], pts[2].screen_pos[2]);
	float depth = uv.dot(z_val);
	if (early_z_test && default_Buffer->depth_buffer->get(p[0], p[1]) < depth)
		return;

	shader->triangle_points = pts;
	shader->frag_bar = uv;
	shader->frag_idx = Vec2i(p[0], p[1]);
	Vec4f color;
	if (!shader->fragment(color))
	{
		
		bool z_passing = true;
		if (z_test)
			z_passing = default_Buffer->depth_buffer->get(p[0], p[1]) > depth;

		if (z_passing)
		{
			if (z_write)
				default_Buffer->depth_buffer->set(p[0], p[1], 0, depth);

			default_Buffer->color_buffer->set(p[0], p[1], 0, color);
			if (MRT)
				shader->MRT();
		}
	}
}

void Renderer::msaa_process(IShader* shader, Point* pts, Vec3f p) {
	bool cover[MSAA_FACTOR];
	float sample_z[MSAA_FACTOR];
	bool not_cover = true;
	Vec3f z_val(pts[0].screen_pos[2], pts[1].screen_pos[2], pts[2].screen_pos[2]);
	for (int s = 0; s < msaa_factor; s++) {
		Vec3f sample = msaa_sample(p, s);
		Vec3f uv = barycentric_gl(pts[0].screen_pos, pts[0].screen_pos, pts[0].screen_pos, sample);
		if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0) {
			cover[s] = false;
			continue;
		}
		cover[s] = true;
		not_cover = false;
		sample_z[s] = uv.dot(z_val);//screen坐标中的z值可以直接线性插值
	}

	if (not_cover)
		return;

	Vec3f pixel(p[0] + 0.5, p[1] + 0.5, 0);
	Vec3f uv = barycentric_gl(pts[0].screen_pos, pts[0].screen_pos, pts[0].screen_pos, pixel);

	shader->triangle_points = pts;
	shader->frag_bar = uv;
	shader->frag_idx = Vec2i(p[0], p[1]);
	Vec4f color;
	if (!shader->fragment(color))
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

				default_Buffer->color_buffer->set(p[0], p[1], s, color);
				if (MRT)
					shader->MRT();
			}
		}
	}
}

Vec4f Renderer::resolve(int x, int y) {
	Vec4f color = Vec4f::Zero();
	for (int i = 0; i < msaa_factor; i++)
	{
		color += default_Buffer->color_buffer->get(x, y, i);
	}
	color /= msaa_factor;
	return color;
}

Vec4f Renderer::gamma_correct(Vec4f color) {
	return Vec4f(std::powf(color[0], gamma_inv),
		std::powf(color[1], gamma_inv),
		std::powf(color[2], gamma_inv),
		color[3]);
}

Vec4f Renderer::tone_mapping(Vec4f color) {
	// Reinhard色调映射
	return Vec4f(color[0] / (color[0] + 1), color[1] / (color[1] + 1), color[2] / (color[2] + 1), color[3]);
}

void Renderer::post_process(unsigned char* image) {
	for (int x = 0; x < screen_width; x++)
	{
		for (int y = 0; y < screen_height; y++)
		{
			Vec4f color = resolve(x, y);

			if (toneMapping)
				color = tone_mapping(color);
			if (gammaCorrect)
				color = gamma_correct(color);

			int image_idx = (screen_height - y - 1) * screen_width + x;
			image[image_idx * 3] = color[2] * 255;
			image[image_idx * 3 + 1] = color[1] * 255;
			image[image_idx * 3 + 2] = color[0] * 255;
		}
	}
}