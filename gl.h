#pragma once
#include <iostream>
#include "model.h"


struct IShader {
	Model* model;
	bool is_perspective;
	Vec3f z_;
	Vec3f clipping_verts[3];

    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, Vec4f& color) = 0;
	virtual bool fragment_deffered(Vec3f bar, ...) { return false; }

	Vec3f perspective_correct_interpolation(Vec3f bar, Vec3f values[3])
	{
		float z_inv = bar[0] / z_[0] + bar[1] / z_[1] + bar[2] / z_[2];
		Vec3f result_d_z = bar[0] * values[0] / z_[0] + bar[1] * values[1] / z_[1] + bar[2] * values[2] / z_[2];
		Vec3f result = result_d_z / z_inv;
		return result;
	}

	float perspective_correct_interpolation(Vec3f bar, Vec3f values)
	{
		float z_inv = bar[0] / z_[0] + bar[1] / z_[1] + bar[2] / z_[2];
		float result_d_z = bar[0] * values[0] / z_[0] + bar[1] * values[1] / z_[1] + bar[2] * values[2] / z_[2];
		float result = result_d_z / z_inv;
		return result;
	}

	Vec3f interpolation(Vec3f bar, Vec3f values[3])
	{
		if (is_perspective)
			return perspective_correct_interpolation(bar, values);
		else
			return values[0] * bar[0] + values[1] * bar[1] + values[2] * bar[2];
	}

	float interpolation(Vec3f bar, Vec3f values)
	{
		if (is_perspective)
			return perspective_correct_interpolation(bar, values);
		else
			return values.dot(bar);
	}
};

struct FrameBuffer {
	float* depth_buffer;
	unsigned char* color_buffer;
	std::vector<void*> other_buffers;

	FrameBuffer() = default;

	FrameBuffer(int sample_num) {
		depth_buffer = new float[sample_num];
		color_buffer = new unsigned char[sample_num * 4];
	}

	void add_buffer(size_t size) {
		void *buffer = malloc(size);
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
	enum DefferedPass {
		GEOMETRY,
		SHADING
	};
	enum CullingMode {
		FRONT,
		BACK
	};

	Renderer(int _width, int _height, int _viewport_x = 0, int _viewport_y = 0, int _depth = 255, bool _use_msaa = false);

	virtual ~Renderer() {
		if (default_Buffer != nullptr)
			delete default_Buffer;
		if (G_Buffer != nullptr)
			delete G_Buffer;
	}

	bool render(Model* model, IShader* shader, TGAImage& image, bool is_perspective = true);

	bool render(Model* model, IShader* shader, unsigned char* image, bool is_perspective = true);

	void clear_zbuffer() {
		for (int i = 0; i < sample_num; i++)
		{
			default_Buffer->depth_buffer[i] = FLT_MAX;
		}
	}

	void get_zbuffer(float* _zbuffer) { 
		if (msaa_factor > 1) {
			for (int i = 0; i < screen_height; i++)
			{
				for (int j = 0; j < screen_width; j++)
				{
					int idx = i * screen_width + j;
					float z = FLT_MAX;
					for (int s = 0; s < msaa_factor; s++)
					{
						if (z > default_Buffer->depth_buffer[idx * msaa_factor + s])
							z = default_Buffer->depth_buffer[idx * msaa_factor + s];
					}
					_zbuffer[idx] = z;
				}
			}
		}
		else {
			memcpy(_zbuffer, default_Buffer->depth_buffer, sizeof(float) * sample_num);
		}
	}

	void set_ztest(bool v) { z_test = v; }
	void set_zwrite(bool v) { z_write = v; }

	void enable_deffered_rendering() {
		assert(msaa_factor == 1);
		deffered_rendering = true;
		if (G_Buffer == nullptr) {
			G_Buffer = new FrameBuffer();
			G_Buffer->add_buffer(sample_num * 3 * sizeof(float));//position(x,y,z)
			G_Buffer->add_buffer(sample_num * 3 * sizeof(float));//normal(x,y,z)
			G_Buffer->add_buffer(sample_num * 4 * sizeof(float));//diffuse_specular(r,g,b,shiness)
			G_Buffer->add_buffer(sample_num * sizeof(bool));//status
		}
	}

	void close_deffered_rendering() {
		deffered_rendering = false;
		if (G_Buffer != nullptr)
			delete G_Buffer;
	}

	void set_defferPass(DefferedPass pass) {
		defferPass = pass;
	}

	void set_cullingMode(CullingMode mode) {
		cullingMode = mode;
	}

	void debug_GBuffer() {
		Vec3f* pos_buffer = (Vec3f*)G_Buffer->other_buffers[0];
		Vec3f* normal_buffer = (Vec3f*)G_Buffer->other_buffers[1];
		Vec4f* diffuse_buffer = (Vec4f*)G_Buffer->other_buffers[2];

		TGAImage pos_image(screen_width, screen_height, TGAImage::RGB);
		TGAImage normal_image(screen_width, screen_height, TGAImage::RGB);
		TGAImage diffuse_image(screen_width, screen_height, TGAImage::RGB);
		TGAImage specular_image(screen_width, screen_height, TGAImage::GRAYSCALE);

		for (int i = 0; i < screen_height; i++)
		{
			for (int j = 0; j < screen_width; j++)
			{
				int idx = i * screen_width + j;
				Vec3f pos = pos_buffer[idx];
				Vec3f normal = normal_buffer[idx];
				Vec4f diffuse_s = diffuse_buffer[idx];

				pos = (pos + Vec3f::Ones()) / 2;
				normal = (normal + Vec3f::Ones()) / 2;
				
				pos_image.set(j, i, TGAColor(255 * pos[0], 255 * pos[1], 255 * pos[2], 255));
				normal_image.set(j, i, TGAColor(255 * normal[0], 255 * normal[1], 255 * normal[2], 255));
				diffuse_image.set(j, i, TGAColor(255 * diffuse_s[0], 255 * diffuse_s[1], 255 * diffuse_s[2], 255));
				specular_image.set(j, i, TGAColor(diffuse_s[3], 0, 0, 0));
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

private:
	FrameBuffer* default_Buffer;//depth,color(rgba)
	Mat4f viewport_mat;
	int screen_width, screen_height;
	int msaa_factor;
	int sample_num;

	bool z_test;
	bool z_write;
	bool culling_face;
	CullingMode cullingMode;

	bool deffered_rendering;
	DefferedPass defferPass;
	FrameBuffer* G_Buffer; //position(x,y,z), normal(x,y,z), diffuse_specular(r,g,b,shiness)

	bool render(Model* model, IShader* shader, bool is_perspective = true);

	TGAColor resolve(int idx);

	void triangle(Vec3f* pts, IShader* shader);

	void process(IShader* shader, Vec3f* pts, Vec3f p);

	void msaa_process(IShader* shader, Vec3f* pts, Vec3f p);

	void deffered_rendering_process(IShader* shader, Vec3f* pts, Vec3f p);
};

