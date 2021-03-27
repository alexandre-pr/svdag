#include "scene/material/material.h"

class BRDF : public Material {
public:

	inline BRDF(Vec3f albedo = Vec3f(1, 1, 1), float diffuse = 0.5, float specular = 1.0, float roughness = 0.5f) :
		albedo(albedo), diffuse(diffuse), specular(specular),
		roughness(roughness), alpha(pow(roughness, 2)), k(pow((roughness + 1), 2) / 8) {};

	Vec3f evaluateColorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const override; 

	inline const float& get_alpha() const override {
		return alpha;
	};

private:
	float epsilon = 1e-10f;
	Vec3f albedo;
	float diffuse;
	float roughness;
	float specular;

	// Precomputations
	float alpha;
	float k;

};