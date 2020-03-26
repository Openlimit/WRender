#pragma once
#include <vector>
#include "geometry.h"
#include "tgaimage.h"

class Model
{
public:
	Model(const char* filename, const char* diffuse_filename = nullptr,
		const char* normal_filename = nullptr, const char* specular_filename = nullptr);
	virtual ~Model();
	int nverts();
	int nfaces();
	Vec3f vert(int i);
	Vec3f vert(int iface, int nthvert);
	Vec3f normal(int i);
	Vec3f normal(int iface, int nthvert);
	Vec3f text(int i);
	Vec3f text(int iface, int nthvert);
	std::vector<Vec3i> face(int i);

	Vec4f diffuse(Vec3f text_coord);
	Vec3f normal(Vec3f text_coord);
	float specular(Vec3f text_coord);

private:
	std::vector<Vec3f> verts_;
	std::vector<Vec3f> texts_;
	std::vector<Vec3f> normals_;
	std::vector<std::vector<Vec3i>> faces_;
	TGAImage* diffuse_image;
	TGAImage* normal_image;
	TGAImage* specular_image;
};

