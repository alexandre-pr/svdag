#include "ray.h"
#include <algorithm>


using namespace std;

Ray::Ray(const Vec3f& origin, const Vec3f& direction) :
	origin(origin),
	direction(direction) {};

Hit Ray::intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const {
	Hit hit;
	intersectTriangle(p0, p1, p2, hit);
	return hit;
};

bool Ray::intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Hit& hit) const {
	// Faces that aren't facing the ray are still counted (used in the shadows)
	Vec3f e0 = p1 - p0;
	Vec3f e1 = p2 - p0;
	Vec3f n = cross(e0, e1);
	n = normalize(n);

	Vec3f q = cross(direction, e1);
	float a = dot(e0, q);

	if (std::abs(a) < epsilon) {
		hit.is_null = true;
		return false;
	}

	Vec3f s = (origin - p0) / a;
	Vec3f r = cross(s, e0);

	float b0 = dot(s, q);
	float b1 = dot(r, direction);
	float b2 = 1 - b0 - b1;

	if ((b0 < 0) || (b1 < 0) || (b2 < 0)) {
		hit.is_null = true;
		return false;
	}

	float t = dot(e1, r);
	if (t >= 0) {
		hit.distance = t;
		hit.b0 = b2;
		hit.b1 = b0;
		hit.b2 = b1;
		hit.is_null = false;
		return true;  // This inversion was required to get smooth output	
	}
	hit.is_null = true;
	return false;
};

bool Ray::intersectBox(const Vec3f& min_corner, const Vec3f& max_corner, Vec3<bool>& next, float& t, Vec3f& exit) const {
	// r.dir is unit direction vector of ray
	float dirfrac_x = 1.0f / (get_direction()[0] + epsilon);
	float dirfrac_y = 1.0f / (get_direction()[1] + epsilon);
	float dirfrac_z = 1.0f / (get_direction()[2] + epsilon);
	// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
	// r.org is origin of ray
	float t1 = (min_corner[0] - get_origin()[0]) * dirfrac_x;
	float t2 = (max_corner[0] - get_origin()[0]) * dirfrac_x;
	float t3 = (min_corner[1] - get_origin()[1]) * dirfrac_y;
	float t4 = (max_corner[1] - get_origin()[1]) * dirfrac_y;
	float t5 = (min_corner[2] - get_origin()[2]) * dirfrac_z;
	float t6 = (max_corner[2] - get_origin()[2]) * dirfrac_z;

	float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
	float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

	// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
	if (tmax < 0)
	{
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
	{
		return false;
	}

	// The ray starts in the box
	if (tmin < 0) {
		t = tmax;
		exit = get_origin() + t * get_direction();
		return true;
	}

	t = tmin;
	exit = get_origin() + tmax * get_direction();

	return true;
}