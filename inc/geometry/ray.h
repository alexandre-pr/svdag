#pragma once

#include "math/vec3.h"

struct Hit {
	float distance;
	bool is_null;
	float b0, b1, b2;

	Hit() : is_null(true), distance(0), b0(0), b1(0), b2(0) {};
	Hit(float distance, float b0, float b1, float b2) : is_null(false),
		distance(distance),
		b0(b0), b1(b1), b2(b2) {};
};

class Ray {
public:
	Ray(const Vec3f& origin, const Vec3f& direction);

	Hit intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const;

	bool intersectTriangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Hit& hit) const;


	// Todo implement next
	bool intersectBox(const Vec3f& min_corner, const Vec3f& max_corner, float& t) const;

	bool intersectBox(const Vec3f& min_corner, const Vec3f& max_corner, int& exit_direction, float &t, Vec3f& exit) const;

	inline const Vec3f& get_origin() const {return origin;}
	
	inline const Vec3f& get_direction() const {return direction;}
	
protected:
	float epsilon = 1e-10F;
	Vec3f origin;
	Vec3f direction;

};