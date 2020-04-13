#pragma once
#include "geometry.h"
#include "utils.h"

struct Light {
	virtual Vec3f light_dir(Vec3f surface_pos) = 0;
	virtual float f_dist(Vec3f surface_pos)=0;
	virtual float f_dir(Vec3f l)=0;

	Vec3f color;
	Vec3f position;
};

struct PointLight :public Light {
	virtual Vec3f light_dir(Vec3f surface_pos) {
		Vec3f dir = position - surface_pos;
		dir.normalize();
		return dir;
	}

	virtual float f_dist(Vec3f surface_pos) override {
		float dist_2 = (position - surface_pos).squaredNorm();
		return 1. / (dist_2 + 1e-3);
	}

	virtual float f_dir(Vec3f l) override {
		return 1.;
	}
};

struct DirectionLight :public Light {
	Vec3f direction;

	virtual Vec3f light_dir(Vec3f surface_pos) override {
		return direction;
	}
	
	virtual float f_dist(Vec3f surface_pos) override {
		return 1.;
	}

	virtual float f_dir(Vec3f l) override {
		return 1.;
	}
};

struct SpotLight :public Light {
	Vec3f front;
	float inner_cos;
	float outter_cos;

	virtual Vec3f light_dir(Vec3f surface_pos) {
		Vec3f dir = position - surface_pos;
		dir.normalize();
		return dir;
	}

	virtual float f_dist(Vec3f surface_pos) override {
		float dist_2 = (position - surface_pos).squaredNorm();
		return 1. / (dist_2 + 1e-3);
	}

	virtual float f_dir(Vec3f l) override {
		Vec3f l_s = -l;
		float t = (front.dot(l_s) - outter_cos) / (inner_cos - outter_cos);
		t = clamp01(t);
		return t * t;
	}
};
