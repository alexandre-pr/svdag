#include "vec3.h"
#include "ray.h"
#include "mesh.h"
#include <algorithm>
#include <vector>
#include <chrono>
#include <iostream>

using namespace std;
using PrimitiveIterator = std::vector<std::pair<int, int>>::iterator;

static const float aabb_eps = 1e-10f;

class AABB {
public:
	AABB();

	AABB(Vec3f min_corner, Vec3f max_corner);

	bool ray_intersection(const Ray& ray, Vec3f& entry, Vec3f& exit, float& t) const;

	bool triangleIntersecton(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const;

	vector<Vec3f> getVertices() const;

	static AABB get_bbox(const vector<Mesh*>& meshes);
	static AABB get_bbox(const vector<Mesh*>& meshes, PrimitiveIterator begin, PrimitiveIterator end);

	Vec3f max_corner;
	Vec3f min_corner;
};