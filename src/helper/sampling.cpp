#include "helper/sampling.h"

std::pair<float, float> random_sample() {
    float dx = uniform_distribution(generator);
    float dy = uniform_distribution(generator);
    return std::pair<float, float>(dx, dy);
}

Vec3f random_vector(const Vec3f& normal) { // Uniform sampling on the half sphere

    float dx = normal_distribution(generator);
    float dy = normal_distribution(generator);
    float dz = normal_distribution(generator);

    Vec3f result = normalize(Vec3f(dx, dy, dz));
    float d = dot(result, normal);
    if (d < 0)
        result -= 2 * d * normal;
    return result;
}


std::pair<float, float> jittered_sample(int sample_idx, int sample_per_pixel) {
    int n = (int)sqrt(sample_per_pixel);
    float d = 1 / ((float)n);
    float dx = uniform_distribution(generator);
    float dy = uniform_distribution(generator);
    int id = sample_idx % (n * n);
    int i = id / n;
    int j = id - i * n;
    return std::pair<float, float>(i * d + dx, j * d + dy);
}


Vec3f jittered_sample_vector_cosine(int sample_idx, int spp, const Vec3f& normal) {
    // Cosine on theta
    if (spp == 1)
        return normal;
    int n = (int)sqrt(spp);
    float d = 1 / ((float)n);
    float dx = uniform_distribution(generator);
    float dy = uniform_distribution(generator);
    int id = sample_idx % (n * n);
    int i = id / n;
    int j = id - i * n;
    float phi = 3.1415f * d * (j + dx); // phi between 0 and 2pi
    float sint = d * (i + dy); // The sinus follows a uniform distribution
    
    Vec3f tangent = cross(normal, Vec3f(1, 0, 0));
    if (tangent.squaredLength() <= epsilon_sample) {
        tangent = cross(normal, Vec3f(0, 1, 0)); // In case the direction is the wrong one
    }

    tangent = normalize(tangent);
    Vec3f cotangent = cross(normal, tangent);

    Vec3f result = (1 - sint * sint) * normal + sint * cos(phi) * tangent + sint * sin(phi) * cotangent;
    return result;
}

// Since we used GGX normal distribution for the brdf, we can sample theta (of the half vector) along
// https://agraphicsguy.wordpress.com/2015/11/01/sampling-microfacet-brdf/
// TODO: if spp=1, return the reflected vector (it would be better for rough surfaces)
Vec3f importance_sample_vector(const Vec3f& normal, const Vec3f& wo, const float& alpha) {
    float alpha2 = alpha * alpha;
    float dx = uniform_distribution(generator);
    float dy = uniform_distribution(generator);
    float phi = 2 * 3.1415f * dx; // phi between 0 and 2pi
    float theta = acos(sqrt((1 - dy) / (dy * (alpha2 - 1) + 1))); // theta between 0 and pi

    Vec3f tangent = cross(normal, Vec3f(1, 0, 0));
    if (length(tangent) <= epsilon_sample) {
        tangent = cross(normal, Vec3f(0, 1, 0)); // In case the direction is the wrong one
    }

    tangent = normalize(tangent);
    Vec3f cotangent = cross(normal, tangent);

    float sint = sin(theta);

    Vec3f wh = cos(theta) * normal + sint * cos(phi) * tangent + sint * sin(phi) * cotangent;

    return -wo + 2 * dot(wo, wh) * wh;
}