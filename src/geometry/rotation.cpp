#include "geometry/rotation.h"

Rotation::Rotation() {
    Mat3f mat;
    matrix = mat.identity();
};

Rotation::Rotation(const Vec3f& axis, float angle) : axis(axis), angle(angle) {
	float c = std::cos(angle);
	float s = std::sin(angle);
	matrix = Mat3f(
		Vec3f(c, -axis[2] * s, axis[1] * s),
		Vec3f(axis[2] * s, c, -axis[0] * s),
		Vec3f(-axis[1] * s, axis[0] * s, c));
	matrix[0] += axis * axis[0] * (1 - c);
	matrix[1] += axis * axis[1] * (1 - c);
	matrix[2] += axis * axis[2] * (1 - c);
};

Rotation::Rotation(const Mat3f& matrix): matrix(matrix) {};

Vec3f Rotation::apply(const Vec3f& p) const {
	return matrix * p;
};