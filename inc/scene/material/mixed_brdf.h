#include "scene/material/brdf.h"
#include <vector>

class MixedBRDF : public Material {
public:
	inline MixedBRDF() {};
	inline MixedBRDF(BRDF m) {
		BRDFs.push_back(m);
		weights.push_back(1);
	};
	inline MixedBRDF(std::vector<BRDF> BRDFs, std::vector<float> weights) :
		BRDFs(BRDFs), weights(weights) {};

	Vec3f evaluateColorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const override;

private:
	std::vector<BRDF> BRDFs;
	std::vector<float> weights;
};