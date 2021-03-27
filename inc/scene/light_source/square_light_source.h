#include "scene/light_source/light_source.h"
#include "helper/sampling.h"

#pragma once

class SquareLightSource : public LightSource {
private:
	float epsilon = 0.001f;
	Vec3f direction;
	Vec3f up;
	float size;
public:
	inline SquareLightSource() {};

	inline SquareLightSource(Vec3f position, Vec3f direction, Vec3f up, float size = 0.2,
		Vec3f color = Vec3f(1, 1, 1),
		float intensity = 1) :
		LightSource(position, color, intensity), size(size), direction(direction), up(up) {};

	inline SquareLightSource(Vec3f position, Vec3f direction, float size = 0.2,
		Vec3f color = Vec3f(1, 1, 1),
		float intensity = 1) : LightSource(position, color, intensity), size(size), direction(direction)
	{
		up = cross(direction, Vec3f(1, 0, 0));
		up = normalize(up);
		if (abs(up[1])>=epsilon){
			up[1] = up[1] * up[1] / abs(up[1]);
		}
		if (length(up) <= epsilon) {
			up = normalize(cross(direction, Vec3f(0, 0, 1))); // In case the direction is the wrong one
			if (up[1] >= epsilon) {
				up[1] = up[1] * up[1] / abs(up[1]);
			}
		}
	}

	inline const Vec3f get_position() const override {
		std::pair<float, float> sample = random_sample();
		Vec3f side = cross(up, direction);
		return position + (sample.first - 0.5f) * size * side + (sample.second - 0.5f) * size * up;
	}

};