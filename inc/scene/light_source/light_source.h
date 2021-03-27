#include "math/vec3.h"

#pragma once

class LightSource {
public:
	Vec3f position;
	Vec3f color;
	float intensity;

	inline LightSource(const Vec3f& position = Vec3f(0, 0, 0), const Vec3f& color = Vec3f(1, 1, 1), float intensity = 1) :
		position(position),
		color(color),
		intensity(intensity)
	{};

	inline virtual const Vec3f get_position() const = 0;
};