#pragma once
#include <iostream>
#include "model.h"
#include "texture.h"

struct IShader {
	Model* model;
	Point* triangle_points;
	Vec3f frag_bar;
	Vec2i frag_idx;
	void** buffers;

	virtual Point vertex(int iface, int nthvert) = 0;
	virtual bool fragment(Vec4f& color) = 0;
	virtual void MRT() {}

	Vec4f get_clipping_pos() {
		return frag_bar[0] * triangle_points[0].clipping_pos +
			frag_bar[1] * triangle_points[1].clipping_pos +
			frag_bar[2] * triangle_points[2].clipping_pos;
	}

	Vec3f get_ndc_pos() {
		return frag_bar[0] * triangle_points[0].ndc_pos + 
			frag_bar[1] * triangle_points[1].ndc_pos + 
			frag_bar[2] * triangle_points[2].ndc_pos;
	}

	Vec3f get_screen_pos() {
		return frag_bar[0] * triangle_points[0].screen_pos + 
			frag_bar[1] * triangle_points[1].screen_pos + 
			frag_bar[2] * triangle_points[2].screen_pos;
	}

	float get_origin_depth() {
		return frag_bar[0] * triangle_points[0].clipping_pos[3] +
			frag_bar[1] * triangle_points[1].clipping_pos[3] +
			frag_bar[2] * triangle_points[2].clipping_pos[3];
	}

	float get_screen_depth() {
		return frag_bar[0] * triangle_points[0].screen_pos[2] +
			frag_bar[1] * triangle_points[1].screen_pos[2] +
			frag_bar[2] * triangle_points[2].screen_pos[2];
	}

	Vec3f get_values(int idx) {
		Vec3f result_d_z = frag_bar[0] * triangle_points[0].values[idx] / triangle_points[0].clipping_pos[3]
			+ frag_bar[1] * triangle_points[1].values[idx] / triangle_points[1].clipping_pos[3]
			+ frag_bar[2] * triangle_points[2].values[idx] / triangle_points[2].clipping_pos[3];
		float depth = get_origin_depth();
		return result_d_z * depth;
	}
};

struct FrameBuffer {
	Texture1f* depth_buffer;
	Texture4f* color_buffer;
	std::vector<void*> other_buffers;

	FrameBuffer() = default;

	FrameBuffer(int width, int height, int depth=1) {
		depth_buffer = new Texture1f(width, height, depth);
		color_buffer = new Texture4f(width, height, depth);
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

	bool render(Model* model, IShader* shader, unsigned char* image);

	void clear_zbuffer() {
		default_Buffer->depth_buffer->clear(FLT_MAX);
	}

	void clear_colorbuffer() {
		default_Buffer->color_buffer->clear();
	}

	void get_zbuffer(Texture1f* zbuffer) { 
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
					zbuffer->set(j, i, 0, z);
				}
			}
		}
		else {
			default_Buffer->depth_buffer->copyTo(zbuffer);
		}
	}

	void get_colorbuffer(Texture4f* color_buffer) {
		if (msaa_factor > 1) {
			for (int y = 0; y < screen_height; y++)
			{
				for (int x = 0; x < screen_width; x++)
				{
					Vec4f color = resolve(x, y);
					color_buffer->set(x, y, 0, color);
				}
			}
		}
		else {
			default_Buffer->color_buffer->copyTo(color_buffer);
		}
	}

	void set_ztest(bool v) { z_test = v; }
	void set_zwrite(bool v) { z_write = v; }
	void set_gammaCorrect(bool v) { gammaCorrect = v; }
	void set_toneMapping(bool v) { toneMapping = v; }
	void set_cullingMode(CullingMode mode) {
		cullingMode = mode;
	}
	void set_cullingFace(bool v) { culling_face = v; }

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
	
	bool toneMapping;
	float gamma_inv;
	bool gammaCorrect;
	bool early_z_test;
	bool z_test;
	bool z_write;
	bool culling_face;
	CullingMode cullingMode;

	bool MRT;
	FrameBuffer* G_Buffer; //position_depth(x,y,z,d), normal(x,y,z), diffuse_specular(r,g,b,shiness)

	bool render(Model* model, IShader* shader);

	bool FaceCulling(Point* verts);

	bool ClipCulling(Point* verts);

	void triangle(Point* pts, IShader* shader);

	void process(IShader* shader, Point* pts, Vec3f p);

	void msaa_process(IShader* shader, Point* pts, Vec3f p);

	Vec4f gamma_correct(Vec4f color);

	Vec4f tone_mapping(Vec4f color);

	Vec4f resolve(int x, int y);

	void post_process(unsigned char* image);
};

