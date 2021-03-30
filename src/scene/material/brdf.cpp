#include "scene/material/brdf.h"

Vec3f BRDF::evaluateColorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const {
	float nwi = dot(normal, wi);
	if (nwi <= 0) { return Vec3f(); }

	Vec3f wh = normalize(wo + wi);
	float alpha2 = alpha * alpha;

	float nwo = dot(normal, wo);
	float wowh = dot(wo, wh);

	const float D = alpha2 / (3.14f * pow(pow(dot(normal, wh), 2) * (alpha2 - 1) + 1, 2) + epsilon);
	const float G1 = nwo / (nwo * (1 - k) + k + epsilon);
	const float G2 = nwi / (nwi * (1 - k) + k + epsilon);
	const float F = fresnel + (1 - fresnel) * ((float)pow(2, (-5.55473f * wowh - 6.98316f) * wowh));

	const float f_diff = diffuse / 3.14f;
	const float f_spec = D * G1 * G2 * F / (4 * nwi * nwo + epsilon);
	//return Vec3f(F, G1 * G2, D)/(4 * nwi * nwo+eps) * nwi;

	//std::cout << f_diff << std::endl << "s: " << f_spec << std::endl;
	return (f_diff + f_spec) * (color * albedo) * nwi;
};