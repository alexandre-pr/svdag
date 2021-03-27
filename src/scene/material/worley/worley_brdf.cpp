#include "scene/material/worley/worley_brdf.h"

Vec3f WorleyBRDF::hue2rgb(float hue) const
{
	Vec3f out;
	int i = (int)(hue * 6);
	float f = hue * 6 - i;
	float m = 1 - f;
	switch (i) {
	case 0:
		out[0] = 1;
		out[1] = f;
		break;
	case 1:
		out[0] = m;
		out[1] = 1;
		break;
	case 2:
		out[1] = 1;
		out[2] = f;
		break;
	case 3:
		out[1] = m;
		out[2] = 1;
		break;
	case 4:
		out[0] = f;
		out[2] = 1;
	case 5:
		out[0] = 1;
		out[2] = m;
	}
	return out;
};


Vec3f WorleyBRDF::evaluateColorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const {
	float hue = noise.evaluate(position);
	hue = clamp(hue, 0.f, 1.f);

	Vec3f albedo = hue2rgb(hue);

	float angle = dot(normal, wi);
	if (angle <= 0) { return Vec3f(); }

	Vec3f wh = normalize(wo + wi);


	const float D = alpha * alpha / (3.14f * pow(pow(dot(normal, wh), 2) * (alpha * alpha - 1) + 1, 2));
	const float G1 = dot(normal, wo) / (dot(normal, wo) * (1 - k) + k);
	const float G2 = dot(normal, wi) / (dot(normal, wi) * (1 - k) + k);
	const float F = specular + (1 - specular) * ((float)pow(2, (-5.55473f * dot(wo, wh) - 6.98316f) * dot(wo, wh)));

	const float f_diff = diffuse / 3.14f;
	const float f_spec = D * G1 * G2 * F / (4 * dot(normal, wi) * dot(normal, wo) + epsilon);

	//std::cout << f_diff << std::endl << "s: " << f_spec << std::endl;
	return (f_diff + f_spec) * (color * albedo) * dot(normal, wi);
};