#include <utility>
#include <algorithm>
#include<chrono>
#include<iostream>
#include <omp.h>

#include "ray_tracer/ray_tracer.h"

void Pixel::record(Vec3f pixel_color, float sample_weight) {
    color.push_back(pixel_color);
    weight.push_back(sample_weight);
};

Vec3f Pixel::get_color() const {
    size_t n = color.size();
    //size_t n_back = n_records - n;
    Vec3f res = Vec3f();
    float total_weight = 0;
    for (int i = 0; i < n; i++) {
        res += color[i];
        total_weight += weight[i];
    }
    //for (int i = 0; i < n_back; i++) {
    //	res += background_color;
    //}
    return res / total_weight;
}









void RayTracer::rayTrace(const Scene& scene, Image& image, const SVDAG& svdag) const {
    const Vec3f c = scene.camera.get_position();
    const Vec3f dir_y = scene.camera.get_up();
    const Vec3f dir_x = cross(scene.camera.get_direction(), dir_y);

    const float real_width = 2 * std::tanf(scene.camera.get_fov() / 2) * scene.camera.get_near();
    const float real_height = real_width / scene.camera.get_aspect_ratio();
    const Vec3f real_center = scene.camera.get_position() + scene.camera.get_direction() * scene.camera.get_near();

    //std::vector<float> l_buffer(image->width * image->height, std::numeric_limits<float>::max());

    //double l0=0, l1=0, l2 = 0;
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < image.width; i++) {
        Vec3f color, position;
        std::cout << i << "/" << image.width << std::endl;
        for (int j = 0; j < image.height; j++) {
            Pixel pix = Pixel();
            for (int ray_idx = 0; ray_idx < ray_per_pixel; ray_idx++) {
                //std::pair<float, float> sample = random_sample();
                //auto t1 = std::chrono::high_resolution_clock::now();
                std::pair<float, float> sample = jittered_sample(ray_idx, ray_per_pixel); // Works better

                Vec3f p = real_center
                    + real_width * dir_x * ((i + sample.first - image.width * 0.5f) / image.width)
                    + real_height * dir_y * ((image.height * 0.5f - j - sample.second) / image.height);
                Ray ray(c, normalize(p - c));

                if (path(scene, ray, position, color, n_bounce, false, svdag)) { // Primitive intersected
                    color = Vec3f(clamp(color, 0.f, 1.f));
                    pix.record(color);
                }
                else
                    pix.record(scene.background_color);
                /*l0 += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                l1 += std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
                l2 += std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();*/

            }
            image.setPixelColor(i, j, pix.get_color());
        }
    }/*
    std::cout << "process took: "
        << l
        << std::endl;*/
    /*
    std::cout << "process took: "
        << l1
        << std::endl;
    std::cout << "process took: "
        << l2
        << std::endl<<std::endl;*/
};

bool RayTracer::path(const Scene& scene, const Ray& ray,
    Vec3f& position, Vec3f& color, int bounce, bool secondary, const SVDAG& svdag) const {
    Hit_BVH hit = scene.ray_intersection(ray);
    if (!hit.is_null) {
        Vec3i f = scene.meshes[hit.primitive_mesh_idx]->get_face(hit.primitive_idx);
        //std::cout << hit.b1 << hit.b2 << hit.b0 << std::endl;
        Vec3f normal = hit.b0 * scene.meshes[hit.primitive_mesh_idx]->get_normal(f[0])
            + hit.b1 * scene.meshes[hit.primitive_mesh_idx]->get_normal(f[1])
            + hit.b2 * scene.meshes[hit.primitive_mesh_idx]->get_normal(f[2]);
        float l = length(normal);
        assert(l != 0);
        normal /= length(normal);
        position = ray.get_origin() + ray.get_direction() * hit.distance;
        color = shade(scene,
            *scene.materials[scene.mesh_material[hit.primitive_mesh_idx]],
            position,
            normal,
            -ray.get_direction(),
            svdag,
            secondary);
        //std::cout << color << std::endl;
        if (bounce > 0) {
            Vec3f total_bounce_color;
            Vec3f bounce_color, bounce_origin;
            int n_sample = secondary ? 1 : spp;
            for (int i = 0; i < n_sample; i++) {
                Ray bounce_ray(position + epsilon * normal,
                    importance_sample_vector(normal, -ray.get_direction(), 
                    scene.materials[scene.mesh_material[hit.primitive_mesh_idx]]->get_alpha()));
                
                /*Ray bounce_ray(position + epsilon * normal,
                    random_vector(normal));*/
                path(scene, bounce_ray, bounce_origin, bounce_color, bounce - 1, true, svdag);
                total_bounce_color += scene.materials[scene.mesh_material[hit.primitive_mesh_idx]]->evaluateColorResponse(
                    position,
                    normal,
                    bounce_ray.get_direction(),
                    -ray.get_direction(),
                    bounce_color
                );
                //std::cout << bounce<< " "<<bounce_color << std::endl;
            }
            color += total_bounce_color / ((float)n_sample);
        }
        return true;
    }
    else {
        color = scene.background_color; 
        // If we bounce in the void, it is better to use the background color (~ambient color) 
        // Else, occluded spaces would not be "more" lit due to bounces always hitting surfaces
        return false;
    }
}

Vec3f RayTracer::shade(const Scene& scene, const Material& material, const Vec3f& position, 
    const Vec3f& normal, const Vec3f& wo, const SVDAG& svdag, bool secondary) const {
    const size_t n_lights = scene.lights.size();
    Vec3f color;

    for (int l = 0; l < n_lights; l++) {
        Vec3f wi = scene.lights[l]->get_position() - position;
        float di = length(wi);
        wi /= di;
        //Vec3f wi = normalize(scene->lights[l].get_pos() - position);


        Ray ray(position + 16.f *svdag.min_stride * wi, wi);
        // Checking if the light is visible from the point
        //if (!scene.ray_blocked(ray, di)) {
        if (!svdag.shadowRay(ray, di)){
            color += scene.lights[l]->intensity *
                material.evaluateColorResponse(position, normal, wi, wo, scene.lights[l]->color);
        }
    }

    //float AO = ambiantOcclusion(scene, position, normal, svdag, secondary);
    float AO = ambiantOcclusion(position, normal, svdag, secondary);
    color += scene.ambiant_color * material.get_albedo() * AO;

    return Vec3f(clamp(color, 0.f, 1.f));
};

//float RayTracer::ambiantOcclusion(const Scene& scene, const Vec3f& position, const Vec3f& normal, const SVDAG& svdag, bool secondary) const {
float RayTracer::ambiantOcclusion(const Vec3f& position, const Vec3f& normal, const SVDAG& svdag, bool secondary) const {
    float AO = 0;
    if (secondary) {// Only one sample
        Ray ray(position + 16 * dot(svdag.min_stride, normal) * normal, normal);
        float l = exp_distrib(generator); // See sampling.h
        //if (!scene.ray_blocked(ray, l))
        if (!svdag.shadowRay(ray, l))
            return 1;
        else
            return 0;
    }

    for (int i = 0; i < n_sample_ao; i++) {
        Vec3f v = jittered_sample_vector_cosine(i, n_sample_ao, normal);
        Ray ray(position + 16.0f * svdag.min_stride * v, v);
        float l = exp_distrib(generator); // See sampling.h
        if (!svdag.shadowRay(ray, l))
        //if (!scene.ray_blocked(ray, l))
            AO += 1;
    }
    return AO/n_sample_ao;
}