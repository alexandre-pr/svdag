#include "math/vec3.h"
#include <limits>

#pragma once

class Camera {

	float epsilon = 0.0001f;

private:
	Vec3f position;
	Vec3f direction = Vec3f(0,0,-1);
	Vec3f up = Vec3f(0,1,0);
	float fov = 3.14f/4;
	float aspect_ratio = 1.5f;

	float near = 0.1f;
	float far = std::numeric_limits<float>::max();

public:
	inline Camera() {};
	
	inline void set_position(const Vec3f& p) {
		position = p;
	}

	inline const Vec3f& get_position() const {
		return position;
	}

	inline void set_direction(const Vec3f& p) { 
		// Hairy ball theorem: we can't have a continous mapping from a direction to a up vector
		direction = p;
		up = cross(direction, Vec3f(1, 0, 0));
		up = normalize(up);
		if (abs(up[1]) >= epsilon) {
			up[1] = up[1] * up[1] / abs(up[1]);
		}
		if (length(up) <= epsilon) {
			up = normalize(cross(direction, Vec3f(0, 0, 1))); // In case the direction is the wrong one
			if (up[1] >= epsilon) {
				up[1] = up[1] * up[1] / abs(up[1]);
			}
		}
	}

	inline void set_direction(const Vec3f& direction_vector, const Vec3f& up_vector) {
		direction = direction_vector;
		up = up_vector;

	}
	
	inline const Vec3f& get_direction() const {
		return direction;
	}

	inline const Vec3f& get_up() const {
		return up;
	}

	inline void set_fov(float f) {
		fov = f;
	}

	inline const float get_fov() const {
		return fov;
	}

	inline void set_aspect_ratio(float a) {
		aspect_ratio = a;
	}

	inline const float get_aspect_ratio() const {
		return aspect_ratio;
	}

	inline void set_near(float n) {
		near = n;
	}

	inline const float get_near() const {
		return near;
	}

	inline void set_far(float f) {
		far = f;
	}

	inline const float get_far() const {
		return far;
	}
};