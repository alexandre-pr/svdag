#include "segment.h"

Hit Segment::intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const {
	Hit hit;
	intersectTriangle(p0, p1, p2, hit);
	return hit;
};

bool Segment::intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Hit& hit) const {
	bool result = Ray::intersectTriangle(p0, p1, p2, hit);
	if (result && (hit.distance>length)) {
		hit.is_null = true;
		result = false;
	}
	return result;
};
