#pragma once
#include <algorithm>
#include "gl.h"
#include "Light.h"

struct DepthShader :public IShader {
	Mat4f MVP;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Point p;
		p.clipping_pos = MVP * vert4;
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		return false;
	}
};

struct GoochShader :public IShader {
	Light* light;
	Vec3f viewPos;

	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Mat4f model_mat_IT;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Vec4f world_vert = model_mat * vert4;

		Point p;
		p.clipping_pos = project_mat * view_mat * world_vert;
		p.values[0] = model->text(iface, nthvert);
		p.values[1] = world_vert.head(3);
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec3f text = shade_point->values[0];
		Vec3f fragPos = shade_point->values[1];

		Vec3f surface_color = model->diffuse(text).head(3);
		Vec3f viewDir = viewPos - fragPos;
		viewDir.normalize();

		Vec3f normal = model->normal(text);
		Vec4f normal4(normal[0], normal[1], normal[2], 0);
		normal4 = model_mat_IT * normal4;
		Vec3f n = normal4.head(3);
		n.normalize();

		Vec3f l = light->light_dir(fragPos);

		Vec3f r = n * (n.dot(l) * 2.f) - l;  // reflected light
		r.normalize();
		
		Vec3f cool = Vec3f(0, 0, 0.55) + 0.25 * surface_color;
		Vec3f warm = Vec3f(0.3, 0.3, 0) + 0.25 * surface_color;
		Vec3f highlight(2, 2, 2);
		float s = clamp01(100 * r.dot(viewDir) - 97);
		float t = std::fmax(0, n.dot(l));
		Vec3f light_color = light->color * light->f_dist(fragPos) * light->f_dir(l);
		Vec3f shade_color = 0.5 * cool + t * light_color.cwiseProduct(s * highlight + (1 - s) * warm);

		color = Vec4f(shade_color[0], shade_color[1], shade_color[2], 1);
		return false;
	}
};

struct GeometryPassShader :public IShader {
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Mat4f model_mat_IT;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Vec4f world_vert = model_mat * vert4;

		Point p;
		p.clipping_pos = project_mat * view_mat * world_vert;
		p.values[0] = model->text(iface, nthvert);
		p.values[1] = world_vert.head(3);
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		return false;
	}

	virtual void MRT()
	{
		Vec2i frag_idx((int)shade_point->screen_pos[0], (int)shade_point->screen_pos[1]);

		Vec3f text = shade_point->values[0];
		Vec3f fragPos = shade_point->values[1];
		float depth = shade_point->clipping_pos[3];
		Vec3f diffuse_color = model->diffuse(text).head(3);

		Vec3f normal = model->normal(text);
		Vec4f normal4(normal[0], normal[1], normal[2], 0);
		normal4 = model_mat_IT * normal4;
		normal = normal4.head(3);
		normal.normalize();

		float shiness = model->specular(text);

		Texture4f* pos_buffer = (Texture4f*)buffers[0];
		Texture3f* normal_buffer = (Texture3f*)buffers[1];
		Texture4f* diffuse_buffer = (Texture4f*)buffers[2];
		Texture1b* status_buffer = (Texture1b*)buffers[3];

		Vec4f fragPosDepth(fragPos[0], fragPos[1], fragPos[2], depth);
		Vec4f diffuse_s(diffuse_color[0], diffuse_color[1], diffuse_color[2], shiness);

		pos_buffer->set(frag_idx[0], frag_idx[1], 0, fragPosDepth);
		normal_buffer->set(frag_idx[0], frag_idx[1], 0, normal);
		diffuse_buffer->set(frag_idx[0], frag_idx[1], 0, diffuse_s);
		status_buffer->set(frag_idx[0], frag_idx[1], 0, true);
	}
};

struct ShadingPassShader :public IShader {
	std::vector<Light*> lights;
	std::vector<Mat4f> lightMats;
	std::vector<Texture1f*> shadowMaps;
	Vec3f viewPos;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Point p;
		p.clipping_pos = vert4;
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec2i frag_idx((int)shade_point->screen_pos[0], (int)shade_point->screen_pos[1]);

		Texture1b* status_buffer = (Texture1b*)buffers[3];
		if (!status_buffer->get(frag_idx[0], frag_idx[1]))
			return true;

		Texture4f* pos_buffer = (Texture4f*)buffers[0];
		Texture3f* normal_buffer = (Texture3f*)buffers[1];
		Texture4f* diffuse_buffer = (Texture4f*)buffers[2];
		Texture1f* ao_buffer = (Texture1f*)buffers[4];

		Vec4f fragPosDepth = pos_buffer->get(frag_idx[0], frag_idx[1]);
		Vec3f fragPos = fragPosDepth.head(3);
		Vec3f normal = normal_buffer->get(frag_idx[0], frag_idx[1]);
		Vec4f diffuse_s = diffuse_buffer->get(frag_idx[0], frag_idx[1]);
		Vec3f diffuse = diffuse_s.head(3);
		float shiness = diffuse_s[3];

		Vec3f viewDir = viewPos - fragPos;
		viewDir.normalize();

		Vec3f shade_color(0, 0, 0);
		Vec4f fragPos4(fragPos[0], fragPos[1], fragPos[2], 1);
		float ao = SSAO_Blur(ao_buffer, status_buffer, frag_idx[0], frag_idx[1]);
		float ambient = 0.15 * ao;
		for (int i = 0; i < lights.size(); i++)
		{
			Vec3f l = lights[i]->light_dir(fragPos);
			Vec3f h = l + viewDir; //half dir
			h.normalize();

			float diff = std::fmax(normal.dot(l), 0);
			float spec = 0;
			if (diff > 0 && shiness >= 1)
				spec = pow(std::fmax(normal.dot(h), 0.), shiness);

			Vec4f fragPosLightSpace = lightMats[i] * fragPos4;
			float shadow = ShadowCalculation(fragPosLightSpace, shadowMaps[i]);

			Vec3f light_color = lights[i]->color * lights[i]->f_dist(fragPos) * lights[i]->f_dir(l);
			shade_color += light_color.cwiseProduct(diffuse * (ambient + (1. - shadow) * (diff + spec)));
		}

		shade_color = clamp1(shade_color);
		color = Vec4f(shade_color[0], shade_color[1], shade_color[2], 1);
		return false;
	}
};

struct SSAOShader :public IShader {
	Mat4f project_mat;
	Mat4f view_mat;
	std::vector<Vec3f> ssaoKernel;
	std::vector<Vec3f> ssaoNoise;
	float radius;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Point p;
		p.clipping_pos = vert4;
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		return false;
	}

	virtual void MRT()
	{
		Vec2i frag_idx((int)shade_point->screen_pos[0], (int)shade_point->screen_pos[1]);

		Texture1b* status_buffer = (Texture1b*)buffers[3];
		if (!status_buffer->get(frag_idx[0], frag_idx[1]))
			return;

		Texture4f* pos_buffer = (Texture4f*)buffers[0];
		Texture3f* normal_buffer = (Texture3f*)buffers[1];
		Vec4f fragPosDepth = pos_buffer->get(frag_idx[0], frag_idx[1]);
		Vec3f fragPos = fragPosDepth.head(3);
		float depth = fragPosDepth[3];
		Vec3f normal = normal_buffer->get(frag_idx[0], frag_idx[1]);

		int s = std::sqrt(ssaoNoise.size());
		int noise_idx = (frag_idx[1] % s) * s + frag_idx[0] % s;
		Vec3f randomVec = ssaoNoise[noise_idx];

		Vec3f tangent = randomVec - normal * randomVec.dot(normal);
		tangent.normalize();
		Vec3f bitangent = normal.cross(tangent);
		Mat3f TBN;
		TBN.col(0) = tangent;
		TBN.col(1) = bitangent;
		TBN.col(2) = normal;

		float occlusion = 0.0;
		for (int i = 0; i < ssaoKernel.size(); ++i)
		{
			Vec3f sample = TBN * ssaoKernel[i];
			sample = fragPos + sample * radius;
			Vec4f offset(sample[0], sample[1], sample[2], 1.0);
			offset = project_mat * view_mat * offset;
			float offset_depth = offset[3];
			offset = offset / offset[3];
			float sampleDepth = pos_buffer->get_by_uv(offset[0] * 0.5 + 0.5, offset[1] * 0.5 + 0.5)[3];
			bool status = status_buffer->get_by_uv(offset[0] * 0.5 + 0.5, offset[1] * 0.5 + 0.5);
			if (status) {
				float rangeCheck = smoothstep(0.0, 1.0, radius / (abs(depth - sampleDepth) + 1e-4));
				occlusion += (sampleDepth < offset_depth ? 1.0 : 0.0) * rangeCheck;
			}
		}
		occlusion = 1.0 - (occlusion / ssaoKernel.size());
		Texture1f* ao_buffer = (Texture1f*)buffers[4];
		ao_buffer->set(frag_idx[0], frag_idx[1], 0, occlusion);
	}
};

struct SkyBoxShader :public IShader {
	Mat4f view_mat;
	Mat4f project_mat;
	TextureCube<Vec3u>* skybox;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);

		Point point;
		point.clipping_pos = project_mat * view_mat * vert4;
		point.clipping_pos[2] = point.clipping_pos[3];
		point.values[0] = vert;
		return point;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec3f text = shade_point->values[0];
		Vec3u color_u = skybox->get(text[0], text[1], text[2]);

		color = Vec4f(color_u[0] / 255., color_u[1] / 255., color_u[2] / 255., 1);
		return false;
	}
};

struct ReflectShadingPassShader :public IShader {
	Vec3f viewPos;
	TextureCube<Vec3u>* skybox;

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Point p;
		p.clipping_pos = vert4;
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec2i frag_idx((int)shade_point->screen_pos[0], (int)shade_point->screen_pos[1]);

		Texture1b* status_buffer = (Texture1b*)buffers[3];
		if (!status_buffer->get(frag_idx[0], frag_idx[1]))
			return true;

		Texture4f* pos_buffer = (Texture4f*)buffers[0];
		Texture3f* normal_buffer = (Texture3f*)buffers[1];

		Vec4f fragPosDepth = pos_buffer->get(frag_idx[0], frag_idx[1]);
		Vec3f fragPos = fragPosDepth.head(3);
		Vec3f normal = normal_buffer->get(frag_idx[0], frag_idx[1]);

		Vec3f viewDir = viewPos - fragPos;
		viewDir.normalize();

		Vec3f r = normal * (normal.dot(viewDir) * 2.f) - viewDir;  // reflected
		r.normalize();

		Vec3u color_u = skybox->get(r[0], r[1], r[2]);

		color = Vec4f(color_u[0] / 255., color_u[1] / 255., color_u[2] / 255., 1);
		return false;
	}
};