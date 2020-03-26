#include "model.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

Model::Model(const char* filename, const char* diffuse_filename,
	const char* normal_filename, const char* specular_filename) {
	std::ifstream in(filename, std::ios::in);
	if (in.fail())return;
	while (!in.eof()) {
		std::string line;
		std::getline(in, line);
		std::istringstream iss(line.c_str());
		char trash;
		if (!line.compare(0, 2, "v ")) {
			iss >> trash;
			Vec3f vec;
			iss >> vec[0] >> vec[1] >> vec[2];
			verts_.emplace_back(vec);
		}
		else if (!line.compare(0, 3, "vt ")) {
			iss >> trash >> trash;
			Vec3f vec;
			iss >> vec[0] >> vec[1] >> vec[2];
			texts_.emplace_back(vec);
		}
		else if (!line.compare(0, 3, "vn ")) {
			iss >> trash >> trash;
			Vec3f vec;
			iss >> vec[0] >> vec[1] >> vec[2];
			normals_.emplace_back(vec);
		}
		else if (!line.compare(0, 2, "f ")) {
			iss >> trash;
			int p_id, t_id, n_id;
			std::vector<Vec3i> indices;
			while (iss >> p_id >> trash >> t_id >> trash >> n_id) {
				Vec3i index(p_id - 1, t_id - 1, n_id - 1);
				indices.emplace_back(index);
			}
			faces_.emplace_back(indices);
		}
	}
	std::cout << "# v# " << verts_.size() << " f# " << faces_.size() << std::endl;

	if (diffuse_filename != nullptr) {
		diffuse_image = new TGAImage();
		diffuse_image->read_tga_file(diffuse_filename);
		diffuse_image->flip_vertically();
	}
	if (normal_filename != nullptr) {
		normal_image = new TGAImage();
		normal_image->read_tga_file(normal_filename);
		normal_image->flip_vertically();
	}
	if (specular_filename != nullptr) {
		specular_image = new TGAImage();
		specular_image->read_tga_file(specular_filename);
		specular_image->flip_vertically();
	}
}

Model::~Model(){
	if (diffuse_image != nullptr)
		delete diffuse_image;
	if (normal_image != nullptr)
		delete normal_image;
}

int Model::nverts() { return verts_.size(); }

int Model::nfaces() { return faces_.size(); }

Vec3f Model::vert(int i) {return verts_[i]; }

Vec3f Model::vert(int iface, int nthvert)
{
	std::vector<Vec3i> indices = faces_[iface];
	return verts_[indices[nthvert][0]];
}

Vec3f Model::normal(int i) { return normals_[i]; }

Vec3f Model::normal(int iface, int nthvert)
{
	std::vector<Vec3i> indices = faces_[iface];
	return normals_[indices[nthvert][2]];
}

Vec3f Model::text(int i) { return texts_[i]; }

Vec3f Model::text(int iface, int nthvert)
{
	std::vector<Vec3i> indices = faces_[iface];
	return texts_[indices[nthvert][1]];
}

std::vector<Vec3i> Model::face(int i) { return faces_[i]; }

Vec4f Model::diffuse(Vec3f text_coord)
{
	int width = diffuse_image->get_width();
	int height = diffuse_image->get_height();
	TGAColor color = diffuse_image->get(text_coord[0] * width, text_coord[1] * height);
	return Vec4f(color.r / 255., color.g / 255., color.b / 255., color.a / 255.);
}

Vec3f Model::normal(Vec3f text_coord)
{
	int width = normal_image->get_width();
	int height = normal_image->get_height();
	TGAColor color = normal_image->get(text_coord[0] * width, text_coord[1] * height);
	Vec3f n;
	for (int i = 0; i < 3; i++)
		n[2 - i] = (float)color[i] / 255.f * 2.f - 1.f;

	n.normalize();
	return n;
}

float Model::specular(Vec3f text_coord) {
	Vec2i uv(text_coord[0] * specular_image->get_width(), text_coord[1] * specular_image->get_height());
	float spec = specular_image->get(uv[0], uv[1])[0] / 1.f;
	return spec;
}

