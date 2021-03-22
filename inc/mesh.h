#include <fstream>
#include <sstream>
#include <string>
#include <cassert>
#include <vector>

#include "transform.h"
#include "ray.h"

#pragma once

class Mesh{
public:
	Mesh();
	Mesh(std::vector<Vec3f> vertices, std::vector<Vec3i> faces);

	inline void add_vertice(Vec3f vertice) {
		vertices.push_back(vertice);
	}

	inline void add_face(Vec3i face) {
		faces.push_back(face);
	}

	void loadOFF(std::string filename);

	void compute_normals();

	inline int n_faces() const {
		return (int)faces.size();
	}

	inline int n_vertices() const {
		return (int)vertices.size();
	}

	inline const Vec3i& get_face(int i) const {
		return faces[i];
	}

	inline const Vec3f& get_vertice(int i) const {
		return vertices[i];
	}

	inline const Vec3f& get_normal(int i) const {
		return normals[i];
	};

	inline Vec3f get_barycenter(int i) const {
		Vec3i face = get_face(i);
		return (get_vertice(face[0]) + get_vertice(face[1]) + get_vertice(face[2])) / 3;
	};


	inline void set_scale(float s) {
		Transform transform = Transform();
		transform.set_scale(s);
		apply_transform(transform);
	}

	inline void set_translation(Vec3f t) {
		Transform transform = Transform();
		transform.set_translation(t);
		apply_transform(transform);
	}

	inline void set_rotation(Vec3f rotation_axis, float rotation_angle) {
		Transform transform = Transform();
		transform.set_rotation(rotation_axis, rotation_angle);
		apply_transform(transform);
	}

	void apply_transform(const Transform& transform);

private:
	std::vector<Vec3f> vertices;
	std::vector<Vec3i> faces;
	std::vector<Vec3f> normals;
};

Mesh square_primitive(const Vec3f& p00 = Vec3f(-1, 0, -1), const Vec3f& p10 = Vec3f(-1, 0, 1),
	const Vec3f& p01 = Vec3f(1, 0, -1), const Vec3f& p11 = Vec3f(1, 0, 1));