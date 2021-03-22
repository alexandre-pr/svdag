#pragma once

#include "ray.h"

class Segment : public Ray {
public:
	Segment(const Vec3f& origin, const Vec3f& direction, float length);

	Hit intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const;

	bool intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Hit& hit) const;

private:
	float length;
};