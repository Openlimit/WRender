#pragma once
#include <iostream>
#include "model.h"


struct IShader {
    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, Vec4f& color) = 0;
	Model* model;
	bool is_perspective;
	Vec3f z_;
	Vec3f clipping_verts[3];

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

class Renderer {
public:
	Renderer(int _width, int _height, int _viewport_x = 0, int _viewport_y = 0, int _depth = 255, bool _use_msaa = false);

	virtual ~Renderer() {
		if (zbuffer != nullptr)
			delete zbuffer;
		if (color_buffer != nullptr)
			delete color_buffer;
	}

	bool render(Model* model, IShader* shader, TGAImage& image, bool is_perspective = true);

	bool render(Model* model, IShader* shader, unsigned char* image, bool is_perspective = true);

	void clear_zbuffer() {
		for (int i = 0; i < sample_num; i++)
		{
			zbuffer[i] = FLT_MAX;
		}
	}

	void get_zbuffer(float* _zbuffer) { 
		for (int i = 0; i < screen_height; i++)
		{
			for (int j = 0; j < screen_width; j++)
			{
				int idx = i * screen_width + j;
				float z = FLT_MAX;
				for (int s = 0; s < msaa_factor; s++)
				{
					if (z > zbuffer[idx * msaa_factor + s])
						z = zbuffer[idx * msaa_factor + s];
				}
				_zbuffer[idx] = z;
			}
		}
	}

private:
	float* zbuffer;
	unsigned char* color_buffer;
	Mat4f viewport_mat;
	int screen_width, screen_height;
	int msaa_factor;
	int sample_num;

	bool render(Model* model, IShader* shader, bool is_perspective = true);

	TGAColor resolve(int idx);

	void triangle(Vec3f* pts, IShader* shader);
};

