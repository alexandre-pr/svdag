#include "scene/material/mixed_brdf.h"


Vec3f MixedBRDF::evaluateColorResponse(const Vec3f& position, const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& color) const{
 Vec3f response;
 for (size_t i = 0; i < BRDFs.size(); i++) {
	 response += BRDFs[i].evaluateColorResponse(position, normal, wi, wo, color) * weights[i];
 }
 return response;
	};