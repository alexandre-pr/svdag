#include "scene/light_source/light_source.h"

#pragma once

class PointLightSource : public LightSource {

	public:
	PointLightSource(const Vec3f& position = Vec3f(0, 0, 0), const Vec3f& color = Vec3f(1, 1, 1), float intensity = 1) :
		LightSource(position, color, intensity)
	{};

	const Vec3f get_position() const override {
		return position;
	}

};
