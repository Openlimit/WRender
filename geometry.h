#pragma once
#include <vector>
#include "utils.h"

#define MAX_POINT_VALUE_NUM 3

Mat4f orthographic(float left, float right, float bottom, float top, float near, float far);

Mat4f perspective(float fov, float wh_ratio, float near, float far);

Mat4f viewport(int x, int y, int width, int height);

Vec3f cal_normal(Vec3f p1, Vec3f p2, Vec3f p3);

struct Point {
	Vec4f clipping_pos;
	Vec3f ndc_pos;
	Vec3f screen_pos;
	Vec3f values[MAX_POINT_VALUE_NUM];

	static Point lerp_point(Point& v1, Point& v2, float w) {
		Point p;
		p.clipping_pos = lerp(v1.clipping_pos, v2.clipping_pos, w);
		for (int i = 0; i < MAX_POINT_VALUE_NUM; i++) {
			p.values[i] = lerp(v1.values[i], v2.values[i], w);
		}
		return p;
	}

	static Point projection_correct_interpolation(Point *points, Vec3f bar) {
		Point p;
		//齐次裁剪空间是线性相关
		p.clipping_pos = bar[0] * points[0].clipping_pos + bar[1] * points[1].clipping_pos + bar[2] * points[2].clipping_pos;
		p.ndc_pos = bar[0] * points[0].ndc_pos + bar[1] * points[1].ndc_pos + bar[2] * points[2].ndc_pos;
		p.screen_pos = bar[0] * points[0].screen_pos + bar[1] * points[1].screen_pos + bar[2] * points[2].screen_pos;

		for (int i = 0; i < MAX_POINT_VALUE_NUM; i++)
		{
			Vec3f result_d_z = bar[0] * points[0].values[i] / points[0].clipping_pos[3]
				+ bar[1] * points[1].values[i] / points[1].clipping_pos[3]
				+ bar[2] * points[2].values[i] / points[2].clipping_pos[3];
			p.values[i] = result_d_z * p.clipping_pos[3];
		}
		return p;
	}
};

std::vector<Point> SutherlandHodgeman(Point& v1, Point& v2, Point& v3);