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
		Vec3f text = get_values(0);
		Vec3f fragPos = get_values(1);

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
		Vec3f text = get_values(0);
		Vec3f fragPos = get_values(1);
		float depth = get_origin_depth();
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

	float ShadowCalculation(Vec4f fragPosLightSpace, Texture1f* shadowMap)
	{
		Vec3f projCoords = fragPosLightSpace.head(3) / fragPosLightSpace[3];
		projCoords = projCoords * 0.5 + Vec3f(0.5, 0.5, 0.5);
		float currentDepth = projCoords[2];
		float bias = 0.005;
		float shadow = 0.0;
		Vec3f texelSize = shadowMap->texel_size();
		for (int x = -1; x <= 1; ++x)
		{
			for (int y = -1; y <= 1; ++y)
			{
				float pcfDepth = shadowMap->get_by_uv(projCoords[0] + x * texelSize[0], projCoords[1] + y * texelSize[1]);
				shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;
			}
		}
		shadow /= 9.0;

		return shadow;
	}

	float SSAO_Blur(Texture1f* ao_buffer, Texture1b* status_buffer, int frag_x, int frag_y) {
		float result = 0.0;
		int count = 0;
		for (int x = -2; x < 2; ++x)
		{
			for (int y = -2; y < 2; ++y)
			{
				int offset_x = frag_x + x;
				int offset_y = frag_y + y;
				if (status_buffer->get(offset_x, offset_y)) {
					result += ao_buffer->get(offset_x, offset_y);
					count++;
				}
			}
		}
		return result / count;
	}

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
		float ambient = 0.3 * ao;
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
		Vec3f text = get_values(0);
		Vec3u color_u = skybox->get(text[0], text[1], text[2]);

		float gamma = 2.2;
		color = Vec4f(
			std::powf(color_u[0] / 255., gamma),
			std::powf(color_u[1] / 255., gamma),
			std::powf(color_u[2] / 255., gamma), 1);
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

struct PBRShader :public IShader {
	Mat4f model_mat;
	Mat4f view_mat;
	Mat4f project_mat;
	Mat4f model_mat_IT;

	Vec3f viewPos;
	Vec3f albedo;
	float metallic;
	float roughness;
	float ao;

	std::vector<Light*> lights;

	float DistributionGGX(Vec3f N, Vec3f H, float roughness)
	{
		float a = roughness * roughness;
		float a2 = a * a;
		float NdotH = std::fmax(N.dot(H), 0.0f);
		float NdotH2 = NdotH * NdotH;

		float nom = a2;
		float denom = (NdotH2 * (a2 - 1.0) + 1.0);
		denom = PI * denom * denom;

		return nom / std::fmax(denom, 0.001f); // prevent divide by zero for roughness=0.0 and NdotH=1.0
	}

	float GeometrySchlickGGX(float NdotV, float roughness)
	{
		float r = (roughness + 1.0);
		float k = (r * r) / 8.0;

		float nom = NdotV;
		float denom = NdotV * (1.0 - k) + k;

		return nom / denom;
	}
	
	float GeometrySmith(Vec3f N, Vec3f V, Vec3f L, float roughness)
	{
		float NdotV = std::fmax(N.dot(V), 0.0f);
		float NdotL = std::fmax(N.dot(L), 0.0f);
		float ggx2 = GeometrySchlickGGX(NdotV, roughness);
		float ggx1 = GeometrySchlickGGX(NdotL, roughness);

		return ggx1 * ggx2;
	}
	
	Vec3f fresnelSchlick(float cosTheta, Vec3f F0)
	{
		return F0 + (Vec3f::Ones() - F0) * pow(1.0 - cosTheta, 5.0);
	}

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);
		Vec4f world_vert = model_mat * vert4;

		Vec3f normal = model->normal(iface, nthvert);
		Vec4f normal4(normal[0], normal[1], normal[2], 0);
		normal4 = model_mat_IT * normal4;

		Point p;
		p.clipping_pos = project_mat * view_mat * world_vert;
		p.values[0] = world_vert.head(3);
		p.values[1] = normal4.head(3);
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec3f fragPos = get_values(0);
		Vec3f V = viewPos - fragPos;
		V.normalize();

		Vec3f N = get_values(1);
		N.normalize();

		// calculate reflectance at normal incidence; if dia-electric (like plastic) use F0 
		// of 0.04 and if it's a metal, use the albedo color as F0 (metallic workflow)  
		Vec3f F0 = Vec3f::Constant(0.04);
		F0 = lerp (F0, albedo, metallic);

		// reflectance equation
		Vec3f Lo = Vec3f::Zero();
		for (int i = 0; i < lights.size(); ++i)
		{
			// calculate per-light radiance
			Vec3f L = lights[i]->light_dir(fragPos);
			Vec3f H = V + L;
			H.normalize();
			
			float attenuation = lights[i]->f_dist(fragPos);
			Vec3f radiance = lights[i]->color * attenuation;

			// Cook-Torrance BRDF
			float NDF = DistributionGGX(N, H, roughness);
			float G = GeometrySmith(N, V, L, roughness);
			Vec3f F = fresnelSchlick(clamp(H.dot(V), 0.0, 1.0), F0);

			Vec3f nominator = NDF * G * F;
			float denominator = 4 * std::fmax(N.dot(V), 0.0f) * std::fmax(N.dot(L), 0.0f);
			Vec3f specular = nominator / std::fmax(denominator, 0.001f); // prevent divide by zero for NdotV=0.0 or NdotL=0.0

			// kS is equal to Fresnel
			Vec3f kS = F;
			// for energy conservation, the diffuse and specular light can't
			// be above 1.0 (unless the surface emits light); to preserve this
			// relationship the diffuse component (kD) should equal 1.0 - kS.
			Vec3f kD = Vec3f::Ones() - kS;
			// multiply kD by the inverse metalness such that only non-metals 
			// have diffuse lighting, or a linear blend if partly metal (pure metals
			// have no diffuse light).
			kD *= 1.0 - metallic;

			// scale light by NdotL
			float NdotL = std::fmax(N.dot(L), 0.0f);

			// add to outgoing radiance Lo
			Lo += (kD.cwiseProduct(albedo) / PI + specular).cwiseProduct(radiance * NdotL);  // note that we already multiplied the BRDF by the Fresnel (kS) so we won't multiply by kS again
		}

		// ambient lighting (note that the next IBL tutorial will replace 
		// this ambient lighting with environment lighting).
		Vec3f ambient = albedo * ao * 0.03; 

		Vec3f shading_color = ambient + Lo;

		shading_color = Vec3f(shading_color[0] / (shading_color[0] + 1),
			shading_color[1] / (shading_color[1] + 1),
			shading_color[2] / (shading_color[2] + 1));

		float gamma_inv = 1. / 2.2;
		shading_color = Vec3f(std::powf(shading_color[0], gamma_inv),
			std::powf(shading_color[1], gamma_inv),
			std::powf(shading_color[2], gamma_inv));

		color = Vec4f(shading_color[0], shading_color[1], shading_color[2], 1);

		return false;
	}
};

struct EquirectangularShader :public IShader {
	Mat4f view_mat;
	Mat4f project_mat;
	Texture3f* enviroment_map_hdr;

	const Vec2f invAtan = Vec2f(0.1591, 0.3183);

	Vec2f SampleSphericalMap(Vec3f v)
	{
		Vec2f uv(std::atan(v[0] / v[2]), std::asin(v[1]));
		uv = uv.cwiseProduct(invAtan) + Vec2f::Constant(0.5);
		return uv;
	}

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);

		Point p;
		p.clipping_pos = project_mat * view_mat * vert4;
		p.values[0] = vert;
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec3f text = get_values(0);
		text.normalize();

		Vec2f uv = SampleSphericalMap(text);
		Vec3f c = enviroment_map_hdr->get_by_uv(uv[0], uv[1]);

		color = Vec4f(c[0] / (c[0] + 1), c[1] / (c[1] + 1), c[2] / (c[2] + 1), 1);
		
		return false;
	}
};