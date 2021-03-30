#include<vector>
#include <iterator>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

#include "scene/camera.h"
#include "scene/light_source/square_light_source.h"
#include "scene/material/mixed_brdf.h"
#include "acceleration_structure/bvh.h"

#pragma once

class Scene
{
public:
	std::vector<Mesh*> meshes;
	std::vector<LightSource*> lights;
	std::vector<Material*> materials;
	std::vector<int> mesh_material;
	Camera camera;
	BVH bvh;
	Vec3f ambiant_color;
	Vec3f background_color;
	

	Scene() : camera(Camera()) {
	};

	Scene(const Vec3f& ambient_color) : ambiant_color(ambient_color), background_color(ambient_color) {};
	Scene(const Vec3f& ambient_color, const Vec3f& background_color) : ambiant_color(ambient_color), background_color(background_color) {};

	void add_mesh(Mesh* mesh) {
		meshes.push_back(mesh);
		mesh_material.push_back(0); // Default material
	}

	void add_light(LightSource* light) {
		lights.push_back(light);
	}

	void add_material(Material* material) {
		materials.push_back(material);
	}

	void set_material(int mesh_idx, int material_idx) {
		mesh_material[mesh_idx] = material_idx;
	}

	int n_meshes() const {
		return (int)meshes.size();
	}
	
	void compute_BVH() {
		bvh = BVH(meshes);
	}

	const Hit_BVH ray_intersection(const Ray& ray) const;

	const bool ray_blocked(const Ray& ray, float max_dist) const;
	// Is the ray towards the light source blocked by a vertice
	// More efficient than using ray_intersection because we don't need the closest vertice

private:
	float eps_scene = 0.000001f;
};