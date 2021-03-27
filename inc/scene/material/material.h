#include "../../math/vec3.h"

#pragma once

//float eps_mat = 0.0000001f;

class Material {
public:

	inline virtual Vec3f evaluateColorResponse(const Vec3f& position, const Vec3f& normal, 
		const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const = 0;
	inline virtual const float& get_alpha() const = 0; // Necessary for importance sampling in BRDF
	// It's easier to modify things here

};