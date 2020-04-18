#pragma once
#include <algorithm>
#include "gl.h"
#include "Light.h"

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
	TextureCube<Vec4f>* irradiance_map_cube;
	TextureCube<Vec4f>* prefilter_map_cube;
	Texture4f* LUT_map;

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

	Vec3f fresnelSchlickRoughness(float cosTheta, Vec3f F0, float roughness)
	{
		Vec3f R = Vec3f::Constant(1 - roughness);
		for (int i = 0; i < 3; i++)
		{
			R[i] = std::fmax(R[i], F0[i]);
		}
		return F0 + (R - F0) * pow(1.0 - cosTheta, 5.0);
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

		Vec3f R = 2.0 * V.dot(N) * N - V;

		// calculate reflectance at normal incidence; if dia-electric (like plastic) use F0 
		// of 0.04 and if it's a metal, use the albedo color as F0 (metallic workflow)  
		Vec3f F0 = Vec3f::Constant(0.04);
		F0 = lerp(F0, albedo, metallic);

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
			Vec3f kD = Vec3f::Ones() - kS;
			kD *= 1.0 - metallic;

			// scale light by NdotL
			float NdotL = std::fmax(N.dot(L), 0.0f);

			// add to outgoing radiance Lo
			Lo += (kD.cwiseProduct(albedo) / PI + specular).cwiseProduct(radiance * NdotL);  // note that we already multiplied the BRDF by the Fresnel (kS) so we won't multiply by kS again
		}

		// ambient lighting (we now use IBL as the ambient term)
		Vec3f F = fresnelSchlickRoughness(std::fmax(N.dot(V), 0.0), F0, roughness);

		Vec3f kS = F;
		Vec3f kD = Vec3f::Ones() - kS;
		kD *= 1.0 - metallic;

		Vec3f irradiance = get_textureCube(irradiance_map_cube, N).head(3);
		Vec3f diffuse = irradiance.cwiseProduct(albedo);

		// sample both the pre-filter map and the BRDF lut and combine them together as per the Split-Sum approximation to get the IBL specular part.
		const float MAX_REFLECTION_LOD = 4.0;
		Vec3f prefilteredColor = get_textureCubeLod(prefilter_map_cube, R, roughness * MAX_REFLECTION_LOD).head(3);
		Vec2f brdf = get_texture2D(LUT_map, std::fmax(N.dot(V), 0.0), roughness).head(2);
		Vec3f specular = prefilteredColor.cwiseProduct(F * brdf[0] + Vec3f::Constant(brdf[1]));

		Vec3f ambient = (kD.cwiseProduct(diffuse) + specular) * ao;

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
		//Vec3f c = enviroment_map_hdr->get_by_uv(uv[0], uv[1]);
		Vec3f c = get_texture2D(enviroment_map_hdr, uv[0], uv[1]);

		color = Vec4f(c[0], c[1], c[2], 1);

		return false;
	}
};

struct EnviromentBoxShader :public IShader {
	Mat4f view_mat;
	Mat4f project_mat;
	TextureCube<Vec4f>* skybox;

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
		//Vec4f color_u = skybox->get(text[0], text[1], text[2]);
		Vec4f color_u = get_textureCube(skybox, text);
		//Vec4f color_u = get_textureCubeLod(skybox, text, 1.2);

		color = Vec4f(color_u[0] / (color_u[0] + 1),
			color_u[1] / (color_u[1] + 1),
			color_u[2] / (color_u[2] + 1), color_u[3]);

		float gamma_inv = 1 / 2.2;
		color = Vec4f(
			std::powf(color[0], gamma_inv),
			std::powf(color[1], gamma_inv),
			std::powf(color[2], gamma_inv), color[3]);
		return false;
	}
};

struct IrradianceConvolutionShader :public IShader {
	Mat4f view_mat;
	Mat4f project_mat;
	TextureCube<Vec4f>* enviroment_map_cube;

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
		Vec3f normal = get_values(0);
		normal.normalize();

		Vec3f irradiance = Vec3f::Zero();

		// tangent space calculation from origin point
		Vec3f up(0.0, 1.0, 0.0);
		Vec3f right = up.cross(normal);
		right.normalize();
		up = normal.cross(right);

		float sampleDelta = 0.025;
		float nrSamples = 0.0;
		for (float phi = 0.0; phi < 2.0 * PI; phi += sampleDelta)
		{
			for (float theta = 0.0; theta < 0.5 * PI; theta += sampleDelta)
			{
				// spherical to cartesian (in tangent space)
				Vec3f tangentSample = Vec3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
				// tangent space to world
				Vec3f sampleVec = tangentSample[0] * right + tangentSample[1] * up + tangentSample[2] * normal;

				Vec3f value = get_textureCube(enviroment_map_cube, sampleVec).head(3);

				irradiance += value * cos(theta) * sin(theta);
				nrSamples++;
			}
		}
		irradiance = PI * irradiance * (1.0 / nrSamples);

		color = Vec4f(irradiance[0], irradiance[1], irradiance[2], 1);

		return false;
	}
};

struct PrefilterShader :public IShader {
	Mat4f view_mat;
	Mat4f project_mat;
	TextureCube<Vec4f>* enviroment_map_cube;
	float roughness;

	float DistributionGGX(Vec3f N, Vec3f H, float roughness)
	{
		float a = roughness * roughness;
		float a2 = a * a;
		float NdotH = std::fmax(N.dot(H), 0.0);
		float NdotH2 = NdotH * NdotH;

		float nom = a2;
		float denom = (NdotH2 * (a2 - 1.0) + 1.0);
		denom = PI * denom * denom;

		return nom / denom;
	}

	// efficient VanDerCorpus calculation.
	float RadicalInverse_VdC(unsigned int bits)
	{
		bits = (bits << 16u) | (bits >> 16u);
		bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
		bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
		bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
		bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
		return float(bits) * 2.3283064365386963e-10; // / 0x100000000
	}
	
	Vec2f Hammersley(unsigned int i, unsigned int N)
	{
		return Vec2f(float(i) / float(N), RadicalInverse_VdC(i));
	}
	
	Vec3f ImportanceSampleGGX(Vec2f Xi, Vec3f N, float roughness)
	{
		float a = roughness * roughness;

		float phi = 2.0 * PI * Xi[0];
		float cosTheta = std::sqrtf((1.0 - Xi[1]) / (1.0 + (a * a - 1.0) * Xi[1]));
		float sinTheta = std::sqrtf(1.0 - cosTheta * cosTheta);

		// from spherical coordinates to cartesian coordinates - halfway vector
		Vec3f H;
		H[0] = cos(phi) * sinTheta;
		H[1] = sin(phi) * sinTheta;
		H[2] = cosTheta;

		// from tangent-space H vector to world-space sample vector
		Vec3f up = std::fabs(N[2]) < 0.999 ? Vec3f(0.0, 0.0, 1.0) : Vec3f(1.0, 0.0, 0.0);
		Vec3f tangent = up.cross(N);
		tangent.normalize();
		Vec3f bitangent = N.cross(tangent);

		Vec3f sampleVec = tangent * H[0] + bitangent * H[1] + N * H[2];
		sampleVec.normalize();
		return sampleVec;
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
		Vec3f N = get_values(0);
		N.normalize();

		// make the simplyfying assumption that V equals R equals the normal 
		Vec3f R = N;
		Vec3f V = R;

		const unsigned int SAMPLE_COUNT = 1024u;
		Vec3f prefilteredColor = Vec3f::Zero();
		float totalWeight = 0.0;

		for (unsigned int i = 0u; i < SAMPLE_COUNT; ++i)
		{
			// generates a sample vector that's biased towards the preferred alignment direction (importance sampling).
			Vec2f Xi = Hammersley(i, SAMPLE_COUNT);
			Vec3f H = ImportanceSampleGGX(Xi, N, roughness);
			Vec3f L = 2.0 * V.dot(H) * H - V;
			L.normalize();

			float NdotL = std::fmax(N.dot(L), 0.0);
			if (NdotL > 0.0)
			{
				// sample from the environment's mip level based on roughness/pdf
				float D = DistributionGGX(N, H, roughness);
				float NdotH = std::fmax(N.dot(H), 0.0);
				float HdotV = std::fmax(H.dot(V), 0.0);
				float pdf = D * NdotH / (4.0 * HdotV) + 0.0001;

				float resolution = 512.0; // resolution of source cubemap (per face)
				float saTexel = 4.0 * PI / (6.0 * resolution * resolution);
				float saSample = 1.0 / (float(SAMPLE_COUNT) * pdf + 0.0001);

				float mipLevel = roughness == 0.0 ? 0.0 : 0.5 * std::log2(saSample / saTexel);
				mipLevel = std::fmin(enviroment_map_cube->textures[0]->maxMipLevel - 1, mipLevel);;

				prefilteredColor += get_textureCubeLod(enviroment_map_cube, L, mipLevel).head(3) * NdotL;
				totalWeight += NdotL;
			}
		}

		prefilteredColor = prefilteredColor / totalWeight;
		color = Vec4f(prefilteredColor[0], prefilteredColor[1], prefilteredColor[2], 1);

		return false;
	}
};

struct LUTShader :public IShader {
	// efficient VanDerCorpus calculation.
	float RadicalInverse_VdC(unsigned int bits)
	{
		bits = (bits << 16u) | (bits >> 16u);
		bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
		bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
		bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
		bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
		return float(bits) * 2.3283064365386963e-10; // / 0x100000000
	}

	Vec2f Hammersley(unsigned int i, unsigned int N)
	{
		return Vec2f(float(i) / float(N), RadicalInverse_VdC(i));
	}

	Vec3f ImportanceSampleGGX(Vec2f Xi, Vec3f N, float roughness)
	{
		float a = roughness * roughness;

		float phi = 2.0 * PI * Xi[0];
		float cosTheta = std::sqrtf((1.0 - Xi[1]) / (1.0 + (a * a - 1.0) * Xi[1]));
		float sinTheta = std::sqrtf(1.0 - cosTheta * cosTheta);

		// from spherical coordinates to cartesian coordinates - halfway vector
		Vec3f H;
		H[0] = cos(phi) * sinTheta;
		H[1] = sin(phi) * sinTheta;
		H[2] = cosTheta;

		// from tangent-space H vector to world-space sample vector
		Vec3f up = std::fabs(N[2]) < 0.999 ? Vec3f(0.0, 0.0, 1.0) : Vec3f(1.0, 0.0, 0.0);
		Vec3f tangent = up.cross(N);
		tangent.normalize();
		Vec3f bitangent = N.cross(tangent);

		Vec3f sampleVec = tangent * H[0] + bitangent * H[1] + N * H[2];
		sampleVec.normalize();
		return sampleVec;
	}

	float GeometrySchlickGGX(float NdotV, float roughness)
	{
		// note that we use a different k for IBL
		float a = roughness;
		float k = (a * a) / 2.0;

		float nom = NdotV;
		float denom = NdotV * (1.0 - k) + k;

		return nom / denom;
	}

	float GeometrySmith(Vec3f N, Vec3f V, Vec3f L, float roughness)
	{
		float NdotV = std::fmax(N.dot(V), 0.0);
		float NdotL = std::fmax(N.dot(L), 0.0);
		float ggx2 = GeometrySchlickGGX(NdotV, roughness);
		float ggx1 = GeometrySchlickGGX(NdotL, roughness);

		return ggx1 * ggx2;
	}

	Vec2f IntegrateBRDF(float NdotV, float roughness)
	{
		Vec3f V;
		V[0] = std::sqrtf(1.0 - NdotV * NdotV);
		V[1] = 0.0;
		V[2] = NdotV;

		float A = 0.0;
		float B = 0.0;

		Vec3f N = Vec3f(0.0, 0.0, 1.0);

		const unsigned int SAMPLE_COUNT = 1024u;
		for (unsigned int i = 0u; i < SAMPLE_COUNT; ++i)
		{
			// generates a sample vector that's biased towards the
			// preferred alignment direction (importance sampling).
			Vec2f Xi = Hammersley(i, SAMPLE_COUNT);
			Vec3f H = ImportanceSampleGGX(Xi, N, roughness);
			Vec3f L = 2.0 * V.dot(H) * H - V;
			L.normalize();

			float NdotL = std::fmax(L[2], 0.0);
			float NdotH = std::fmax(H[2], 0.0);
			float VdotH = std::fmax(V.dot(H), 0.0);

			if (NdotL > 0.0)
			{
				float G = GeometrySmith(N, V, L, roughness);
				float G_Vis = (G * VdotH) / (NdotH * NdotV);
				float Fc = std::pow(1.0 - VdotH, 5.0);

				A += (1.0 - Fc) * G_Vis;
				B += Fc * G_Vis;
			}
		}
		A /= float(SAMPLE_COUNT);
		B /= float(SAMPLE_COUNT);
		return Vec2f(A, B);
	}

	virtual Point vertex(int iface, int nthvert)
	{
		Vec3f vert = model->vert(iface, nthvert);
		Vec4f vert4(vert[0], vert[1], vert[2], 1);

		Point p;
		p.clipping_pos = vert4;
		p.values[0] = model->text(iface, nthvert);
		return p;
	}

	virtual bool fragment(Vec4f& color)
	{
		Vec3f text = get_values(0);
		Vec2f integratedBRDF = IntegrateBRDF(text[0], text[1]);
		color = Vec4f(integratedBRDF[0], integratedBRDF[1], 0, 1);

		return false;
	}
};