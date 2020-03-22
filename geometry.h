#pragma once
#include <Eigen/Dense>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix4f Mat4f;

#define PI 3.1415926

Mat4f orthographic(float left, float right, float bottom, float top, float near, float far);

Mat4f perspective(float fov, float wh_ratio, float near, float far);

Mat4f viewport(int x, int y, int width, int height, int depth);