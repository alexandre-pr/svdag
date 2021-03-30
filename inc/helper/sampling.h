#include <utility>
#include <random>
#include "math/vec3.h"

#pragma once

static float epsilon_sample;

// Sample in [0,1]^2 using different methods

static std::default_random_engine generator;
static std::normal_distribution<float> normal_distribution(0, 1);
static std::uniform_real_distribution<float> uniform_distribution(0, 1);

std::pair<float, float> random_sample();

Vec3f random_vector(const Vec3f& normal);

std::pair<float, float> jittered_sample(int sample_idx, int sample_per_pixel);

//Vec3f jittered_sample_vector(int sample_idx, int spp, const Vec3f& normal);

Vec3f jittered_sample_vector_cosine(int sample_idx, int spp, const Vec3f& normal);


// Since we used GGX normal distribution for the brdf, we can sample theta (of the half vector) along
// https://agraphicsguy.wordpress.com/2015/11/01/sampling-microfacet-brdf/
Vec3f importance_sample_vector(const Vec3f& normal, const Vec3f& wo, const float& alpha);