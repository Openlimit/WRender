#include "geometry.h"

Mat4f orthographic(float left, float right, float bottom, float top, float near, float far)
{
	Mat4f scale;
	scale << 2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, 2 / (far - near), 0,
		0, 0, 0, 1;
	Mat4f translate;
	translate << 1, 0, 0, -(right + left) / 2,
		0, 1, 0, -(top + bottom) / 2,
		0, 0, 1, -(far + near) / 2,
		0, 0, 0, 1;
	Mat4f project = scale * translate;
	return project;
}

Mat4f perspective(float fov, float wh_ratio, float near, float far) {
	float half_height = std::tanf(fov / 2) * near;
	float half_width = half_height * wh_ratio;
	Mat4f orth = orthographic(-half_width, half_width, -half_height, half_height, near, far);
	Mat4f pers;
	pers << near, 0, 0, 0,
		0, near, 0, 0,
		0, 0, near + far, -near * far,
		0, 0, 1, 0;
	pers = orth * pers;
	return pers;  
}

Mat4f viewport(int x, int y, int width, int height) {
	Mat4f m;
	m << width / 2.f, 0, 0, width / 2.f + x,
		0, height / 2.f, 0, height / 2.f + y,
		0, 0, 1 / 2.f, 1 / 2.f,
		0, 0, 0, 1;
	return m;
}

Vec3f cal_normal(Vec3f p1, Vec3f p2, Vec3f p3) {
	Vec3f AB = p2 - p1;
	Vec3f AC = p3 - p1;
	Vec3f n = AC.cross(AB);
	n.normalize();
	return n;
}

bool Inside(Vec4f line, Point& point) {
	Vec4f p = point.clipping_pos;
	return line[0] * p[0] + line[1] * p[1] + line[2] * p[2] + line[3] * p[3] >= 0;
}

//交点，通过端点插值
Point Intersect(Point& v1, Point& v2, Vec4f line) {
	Vec4f vp1 = v1.clipping_pos;
	Vec4f vp2 = v2.clipping_pos;
	float da = vp1[0] * line[0] + vp1[1] * line[1] + vp1[2] * line[2] + line[3] * vp1[3];
	float db = vp2[0] * line[0] + vp2[1] * line[1] + vp2[2] * line[2] + line[3] * vp2[3];

	float weight = da / (da - db);
	return Point::lerp_point(v1, v2, weight);
}

const std::vector<Vec4f> ViewLines = {
	//Near
	Vec4f(0,0,1,1),
	//far
	Vec4f(0,0,-1,1),
	//left
	Vec4f(1,0,0,1),
	//top
	Vec4f(0,1,0,1),
	//right
	Vec4f(-1,0,0,1),
	//bottom 
	Vec4f(0,-1,0,1)
};

bool AllVertexsInside(std::vector<Point>& input) {
	for (int i = 0; i < ViewLines.size(); i++)
	{
		for (int j = 0; j < input.size(); j++)
		{
			if (!Inside(ViewLines[i], input[j]))
				return false;
		}
	}
	return true;
}

std::vector<Point> SutherlandHodgeman(Point& v1, Point& v2, Point& v3) {
	std::vector<Point> output = { v1,v2,v3 };
	if (AllVertexsInside(output)) {
		return output;
	}
	for (int i = 0; i < ViewLines.size(); i++) {
		std::vector<Point> input(output);
		output.clear();
		for (int j = 0; j < input.size(); j++) {
			Point current = input[j];
			Point last = input[(j + input.size() - 1) % input.size()];
			if (Inside(ViewLines[i], current)) {
				if (!Inside(ViewLines[i], last)) {
					Point intersecting = Intersect(last, current, ViewLines[i]);
					output.emplace_back(intersecting);
				}
				output.emplace_back(current);
			}
			else if (Inside(ViewLines[i], last)) {
				Point intersecting = Intersect(last, current, ViewLines[i]);
				output.emplace_back(intersecting);
			}
		}
	}
	return output;
}