#include "math/mat3.h"
#pragma once

struct Rotation {
private:
	Vec3f axis;
	float angle;
	Mat3f matrix;
public:
	Rotation();

	Rotation(const Vec3f& axis, float angle);

	Rotation(const Mat3f& matrix);

	Vec3f apply(const Vec3f& p) const;
};