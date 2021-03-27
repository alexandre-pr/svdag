#include "scene/material/material.h"
#include "scene/material/worley/worley.h"

#pragma once

class WorleyBRDF : public Material {
public:

	inline WorleyBRDF(int N_samples = 100, float diffuse = 1, float specular = 1, float roughness = .5f) :
		diffuse(diffuse), specular(specular),
		roughness(roughness), alpha(pow(roughness, 2)), k(pow((roughness + 1), 2) / 8) {
		noise = Worley(N_samples);
	}

	Vec3f hue2rgb(float hue) const;

	Vec3f evaluateColorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const override;


private:
	float epsilon = 1e-10f;
	float diffuse;
	float roughness;
	float specular;
	Worley noise;

	// Precomputations
	float alpha;
	float k;
};