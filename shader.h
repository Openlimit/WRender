#pragma once
#include <algorithm>
#include "gl.h"


struct PhongShader :public IShader {
	Vec3f light_dir;
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Mat4f model_mat_IT;
	Vec3f viewPos;

	Vec3f text_coord[3];
	Vec3f world_verts[3];
	
	virtual Vec4f vertex(int iface, int nthvert)
	{
		text_coord[nthvert] = model->text(iface, nthvert);

		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Vec4f world_vert = model_mat* vert4;
		world_verts[nthvert] = world_vert.head(3);
		return project_mat * view_mat * world_vert;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)
	{
		Vec3f text = interpolation(bar, text_coord);;
		TGAColor diffuse_color = model->diffuse(text);

		Vec3f fragPos = interpolation(bar, world_verts);
		Vec3f viewDir = viewPos - fragPos;
		viewDir.normalize();

		Vec3f normal = model->normal(text);
		Vec4f normal4(normal[0], normal[1], normal[2], 0);
		normal4 = model_mat_IT * normal4;
		Vec3f n = normal4.head(3);
		n.normalize();

		Vec3f r = n * (n.dot(light_dir) * 2.f) - light_dir;  // reflected light
		r.normalize();
		float spec = pow(std::fmax(r.dot(viewDir), 0.0f), model->specular(text));
		float diff = std::fmax(0, n.dot(light_dir));

		for (int i = 0; i < 3; i++) 
			color[i] = std::fmin(5 + diffuse_color[i] * (diff + .6 * spec), 255);
		return false;
	}
};

struct TextureShader :public IShader {
	Vec3f light_dir;
	Mat4f MVP;
	Mat3f M;
	Mat3f MIT;

	Vec3f text_coord[3];

	virtual Vec4f vertex(int iface, int nthvert)
	{
		text_coord[nthvert] = model->text(iface, nthvert);

		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		return MVP * vert4;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)
	{
		Vec3f text = interpolation(bar, text_coord);
		TGAColor diffuse_color = model->diffuse(text);

		Vec3f normal = model->normal(text);
		Vec3f n = MIT * normal;
		n.normalize();

		Vec3f l = M * light_dir;
		l.normalize();

		Vec3f r = n * (n.dot(l) * 2.f) - l;  // reflected light
		r.normalize();
		float spec = pow(std::fmax(r[2], 0.0f), model->specular(text));
		float diff = std::fmax(0, n.dot(l));

		for (int i = 0; i < 3; i++)
			color[i] = std::fmin(5 + diffuse_color[i] * (diff + .6 * spec), 255);
		return false;
	}
};

struct GouraudShader : public IShader {
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Vec3f light_dir;

	Vec3f varying_intensity; 

	virtual Vec4f vertex(int iface, int nthvert) {
		varying_intensity[nthvert] = std::fmax(0.f, model->normal(iface, nthvert).dot(light_dir)); 
		
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		return project_mat * view_mat * model_mat * vert4;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color) {
		float intensity = interpolation(bar, varying_intensity);
		if (intensity > .85) intensity = 1;
		else if (intensity > .60) intensity = .80;
		else if (intensity > .45) intensity = .60;
		else if (intensity > .30) intensity = .45;
		else if (intensity > .15) intensity = .30;
		else intensity = 0;
		color = TGAColor(255 * intensity, 255 * intensity, 255 * intensity, 255);
		return false;                              
	}
};

struct DepthShader :public IShader {
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;

	Vec3f z_arr;

	virtual Vec4f vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Vec4f clipping_pos = project_mat * view_mat * model_mat * vert4;
		z_arr[nthvert] = clipping_pos[2] / clipping_pos[3];
		return clipping_pos;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)
	{
		float z = interpolation(bar, z_arr);
		float d = (z + 1) / 2;
		color = TGAColor(255 * d, 255 * d, 255 * d, 255);
		return false;
	}
};

struct ShadowShader :public IShader {
	Vec3f light_dir;
	Vec3f viewPos;

	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Mat4f model_mat_IT;
	
	int width;
	int height;
	float* shadowBuffer;
	Mat4f shadow_M;

	Vec3f text_coord[3];
	Vec3f world_verts[3];

	virtual Vec4f vertex(int iface, int nthvert)
	{
		text_coord[nthvert] = model->text(iface, nthvert);

		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Vec4f world_vert = model_mat * vert4;
		world_verts[nthvert] = world_vert.head(3);
		return project_mat * view_mat * world_vert;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)
	{
		Vec3f text = interpolation(bar, text_coord);
		TGAColor diffuse_color = model->diffuse(text);

		Vec3f fragPos = interpolation(bar, world_verts);
		Vec3f viewDir = viewPos - fragPos;
		viewDir.normalize();

		Vec3f normal = model->normal(text);
		Vec4f normal4(normal[0], normal[1], normal[2], 0);
		normal4 = model_mat_IT * normal4;
		Vec3f n = normal4.head(3);
		n.normalize();

		Vec3f r = n * (n.dot(light_dir) * 2.f) - light_dir;  // reflected light
		r.normalize();
		float spec_f = model->specular(text);
		float spec;
		if (spec_f < 1)
			spec = 0;
		else
			spec = pow(std::fmax(r.dot(viewDir), 0.0f), spec_f);
		float diff = std::fmax(0, n.dot(light_dir));

		Vec3f pos = interpolation(bar, clipping_verts);
		Vec4f pos4(pos[0], pos[1], pos[2], 1);
		Vec4f shadow_pos = shadow_M * pos4;
		shadow_pos = shadow_pos / shadow_pos[3];

		int x = (shadow_pos[0] + 1.) * width / 2.;
		int y = (shadow_pos[1] + 1.) * height / 2.;
		int idx = y * width + x;
		float shadow=1;
		if (shadowBuffer[idx] < shadow_pos[2] - 1e-2)
			shadow = 0.3;
		else
			shadow = 1;

		for (int i = 0; i < 3; i++)
			color[i] = std::fmin(10 + diffuse_color[i] * shadow * (1.2 * diff + .6 * spec), 255);
		return false;
	}
};