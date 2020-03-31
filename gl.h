#pragma once
#include <iostream>
#include "model.h"
#include "texture.h"

struct IShader {
	Model* model;
	Vec3f z_invs_;
	Vec3f ndc_verts[3];
	Vec2i frag_idx;
	void** buffers;

    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, Vec4f& color) = 0;
	virtual void MRT(Vec3f bar) {}

	Vec3f interpolation(Vec3f bar, Vec3f values[3])
	{
		float z_inv = bar[0] * z_invs_[0] + bar[1] * z_invs_[1] + bar[2] * z_invs_[2];
		Vec3f result_d_z = bar[0] * values[0] * z_invs_[0] + bar[1] * values[1] * z_invs_[1] + bar[2] * values[2] * z_invs_[2];
		Vec3f result = result_d_z / z_inv;
		return result;
	}

	float interpolation(Vec3f bar, Vec3f values)
	{
		float z_inv = bar[0] * z_invs_[0] + bar[1] * z_invs_[1] + bar[2] * z_invs_[2];
		float result_d_z = bar[0] * values[0] * z_invs_[0] + bar[1] * values[1] * z_invs_[1] + bar[2] * values[2] * z_invs_[2];
		float result = result_d_z / z_inv;
		return result;
	}

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
};

struct FrameBuffer {
	Texture1f* depth_buffer;
	Texture4u* color_buffer;
	std::vector<void*> other_buffers;

	FrameBuffer() = default;

	FrameBuffer(int width, int height, int depth=1) {
		depth_buffer = new Texture1f(width, height, depth);
		color_buffer = new Texture4u(width, height, depth);
	}

	void add_buffer(void* buffer) {
		other_buffers.emplace_back(buffer);
	}

	virtual ~FrameBuffer() {
		if (depth_buffer != nullptr)
			delete depth_buffer;
		if (color_buffer != nullptr)
			delete color_buffer;
		for (int i = 0; i < other_buffers.size(); i++)
		{
			if (other_buffers[i] != nullptr)
				delete other_buffers[i];
		}
	}
};

class Renderer {
public:
	enum CullingMode {
		FRONT,
		BACK
	};

	Renderer(float _near, float _far, int _width, int _height, int _viewport_x = 0, int _viewport_y = 0, bool _use_msaa = false);

	virtual ~Renderer() {
		if (default_Buffer != nullptr)
			delete default_Buffer;
		if (G_Buffer != nullptr)
			delete G_Buffer;
	}

	bool render(Model* model, IShader* shader, TGAImage& image);

	bool render(Model* model, IShader* shader, unsigned char* image);

	void clear_zbuffer() {
		default_Buffer->depth_buffer->clear(FLT_MAX);
	}

	void get_zbuffer(Texture1f* _zbuffer) { 
		if (msaa_factor > 1) {
			for (int i = 0; i < screen_height; i++)
			{
				for (int j = 0; j < screen_width; j++)
				{
					float z = FLT_MAX;
					for (int s = 0; s < msaa_factor; s++)
					{
						float t_z = default_Buffer->depth_buffer->get(j, i, s);
						if (z > t_z)
							z = t_z;
					}
					_zbuffer->set(j, i, 0, z);
				}
			}
		}
		else {
			default_Buffer->depth_buffer->copyTo(_zbuffer);
		}
	}

	void set_ztest(bool v) { z_test = v; }
	void set_zwrite(bool v) { z_write = v; }

	void enable_deffered_rendering() {
		assert(msaa_factor == 1);
		MRT = true;
		if (G_Buffer == nullptr) {
			Texture4f* pos_buffer = new Texture4f(screen_width, screen_height);
			Texture3f* normal_buffer = new Texture3f(screen_width, screen_height);
			Texture4f* diffuse_buffer = new Texture4f(screen_width, screen_height);
			Texture1b* status_buffer = new Texture1b(screen_width, screen_height);
			Texture1f* ao_buffer = new Texture1f(screen_width, screen_height);

			G_Buffer = new FrameBuffer();
			G_Buffer->add_buffer(pos_buffer);//position(x,y,z,d)
			G_Buffer->add_buffer(normal_buffer);//normal(x,y,z)
			G_Buffer->add_buffer(diffuse_buffer);//diffuse_specular(r,g,b,shiness)
			G_Buffer->add_buffer(status_buffer);//status
			G_Buffer->add_buffer(ao_buffer);//SSAO
		}
	}

	void close_deffered_rendering() {
		MRT = false;
		if (G_Buffer != nullptr)
			delete G_Buffer;
	}

	void clear_deffered_rendering() {
		assert(G_Buffer != nullptr && G_Buffer->other_buffers[3] != nullptr);
		((Texture1b*)G_Buffer->other_buffers[3])->clear();
	}

	void set_cullingMode(CullingMode mode) {
		cullingMode = mode;
	}

	bool FaceCulling(Vec3f* verts);

	bool ClipCulling(Vec4f* verts);

	void debug_GBuffer() {
		Texture4f* pos_buffer = (Texture4f*)G_Buffer->other_buffers[0];
		Texture3f* normal_buffer = (Texture3f*)G_Buffer->other_buffers[1];
		Texture4f* diffuse_buffer = (Texture4f*)G_Buffer->other_buffers[2];

		TGAImage pos_image(screen_width, screen_height, TGAImage::RGB);
		TGAImage normal_image(screen_width, screen_height, TGAImage::RGB);
		TGAImage diffuse_image(screen_width, screen_height, TGAImage::RGB);
		TGAImage specular_image(screen_width, screen_height, TGAImage::GRAYSCALE);

		for (int i = 0; i < screen_height; i++)
		{
			for (int j = 0; j < screen_width; j++)
			{
				Vec4f fragPosDepth = pos_buffer->get(j, i);
				Vec3f fragPos = fragPosDepth.head(3);
				Vec3f normal = normal_buffer->get(j, i);
				Vec4f diffuse_s = diffuse_buffer->get(j, i);
				Vec3f diffuse = diffuse_s.head(3);
				float shiness = diffuse_s[3];

				fragPos = (fragPos + Vec3f::Ones()) / 2;
				normal = (normal + Vec3f::Ones()) / 2;
				
				pos_image.set(j, i, TGAColor(255 * fragPos[0], 255 * fragPos[1], 255 * fragPos[2], 255));
				normal_image.set(j, i, TGAColor(255 * normal[0], 255 * normal[1], 255 * normal[2], 255));
				diffuse_image.set(j, i, TGAColor(255 * diffuse_s[0], 255 * diffuse_s[1], 255 * diffuse_s[2], 255));
				specular_image.set(j, i, TGAColor(0, 0, shiness, 0));
			}
		}
		pos_image.flip_vertically();
		pos_image.write_tga_file("pos_map.tga");
		normal_image.flip_vertically();
		normal_image.write_tga_file("normal_map.tga");
		diffuse_image.flip_vertically();
		diffuse_image.write_tga_file("diffuse_map.tga");
		specular_image.flip_vertically();
		specular_image.write_tga_file("specular_map.tga");
	}

	void debug_zbuffer() {
		TGAImage image(screen_width, screen_height, TGAImage::GRAYSCALE);
		for (int i = 0; i < screen_height; i++)
		{
			for (int j = 0; j < screen_width; j++)
			{
				float depth = default_Buffer->depth_buffer->get(j, i);
				image.set(j, i, TGAColor(0, 0, 255 * depth, 0));
			}
		}
		image.flip_vertically();
		image.write_tga_file("depth_map.tga");
	}

private:
	FrameBuffer* default_Buffer;//depth,color(rgba)
	Mat4f viewport_mat;
	float dnear;
	float dfar;
	int screen_width, screen_height;
	int msaa_factor;
	
	bool z_test;
	bool z_write;
	bool culling_face;
	CullingMode cullingMode;

	bool MRT;
	FrameBuffer* G_Buffer; //position_depth(x,y,z,d), normal(x,y,z), diffuse_specular(r,g,b,shiness)

	bool render(Model* model, IShader* shader);

	TGAColor resolve(int x, int y);

	void triangle(Vec3f* pts, IShader* shader);

	void process(IShader* shader, Vec3f* pts, Vec3f p);

	void msaa_process(IShader* shader, Vec3f* pts, Vec3f p);
};

