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
	inline RayTracer(int ray_per_pixel = 16, int bounce = 3, int spp = 4, int n_sample_ao=16, float ao_falloff_distance = 0.1f) : 
		ray_per_pixel(ray_per_pixel), n_bounce(bounce), spp(spp), n_sample_ao(n_sample_ao), ao_falloff_distance(ao_falloff_distance), exp_distrib(1/ao_falloff_distance) {};

	void rayTrace(const Scene& scene, Image& image, const SVDAG& svdag) const;
	bool path(const Scene& scene, const Ray& ray,
		Vec3f& position, Vec3f& color, int bounce, bool secondary, const SVDAG& svdag) const;
	Vec3f shade(const Scene& scene, const Material& material, const Vec3f& position, const Vec3f& normal, const Vec3f& wo, const SVDAG& svdag, bool secondary=false) const;
	//float ambiantOcclusion(const Vec3f& position, const Vec3f& normal, const SVDAG& svdag, bool secondary=false) const;
	float ambiantOcclusion(const Scene& scene, const Vec3f& position, const Vec3f& normal, const SVDAG& svdag, bool secondary) const;

private:
	int ray_per_pixel;
	int spp; // sample path per pixel
	int n_bounce;
	int n_sample_ao;
	float ao_falloff_distance;
	exponential_distribution<float> exp_distrib;
};