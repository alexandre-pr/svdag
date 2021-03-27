#pragma once

#include "input_output/image.h"
#include "scene/scene.h"
#include "geometry/ray.h"
#include "helper/sampling.h"
#include "acceleration_structure/svdag.h"

struct Pixel {
private:
	std::vector<Vec3f> color;
	std::vector<float> weight;

public:
	inline Pixel() {};

	void record(Vec3f pixel_color, float sample_weight = 1);

	Vec3f get_color() const;
};


class RayTracer {
	float epsilon = 0.00001f;
public:
	inline RayTracer(int ray_per_pixel = 16, int bounce = 3, int spp = 4) : ray_per_pixel(ray_per_pixel), n_bounce(bounce), spp(spp) {};

	void rayTrace(const Scene& scene, Image& image, const SVDAG& svdag) const;
	void path(const Scene& scene, const Ray& ray,
		Vec3f& position, Vec3f& color, int bounce, int spp_used, const SVDAG& svdag) const;
	Vec3f shade(const Scene& scene, const Material& material, const Vec3f& position, const Vec3f& normal, const Vec3f& wo, const SVDAG& svdag) const;

private:
	int ray_per_pixel;
	int spp; // sample path per pixel
	int n_bounce;
};