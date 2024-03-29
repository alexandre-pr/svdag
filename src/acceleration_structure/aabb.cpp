#include "acceleration_structure/aabb.h"
#include <limits>

float AABB::aabb_eps = 1e-10f;

AABB::AABB() {};

AABB::AABB(Vec3f min_corner, Vec3f max_corner) : min_corner(min_corner), max_corner(max_corner + Vec3f(aabb_eps, aabb_eps, aabb_eps)) {
};

vector<Vec3f> AABB::getVertices() const {
    vector<Vec3f> result;
    result.push_back(min_corner);
    result.push_back(Vec3f(min_corner[0], min_corner[1], max_corner[2]));
    result.push_back(Vec3f(min_corner[0], max_corner[1], min_corner[2]));
    result.push_back(Vec3f(min_corner[0], max_corner[1], max_corner[2]));
    result.push_back(Vec3f(max_corner[0], min_corner[1], min_corner[2]));
    result.push_back(Vec3f(max_corner[0], min_corner[1], max_corner[2]));
    result.push_back(Vec3f(max_corner[0], max_corner[1], min_corner[2]));
    result.push_back(max_corner);
	return result;
};

AABB AABB::get_bbox(const vector<Mesh*>& meshes, PrimitiveIterator begin, PrimitiveIterator end) {
	assert(begin != end);

	pair<int, int> primitive = *begin;
	Vec3f min_corner = meshes[primitive.first]->get_vertice(meshes[primitive.first]->get_face(primitive.second)[0]);
	Vec3f max_corner = min_corner;
	for (PrimitiveIterator it = begin; it != end; ++it) {
		primitive = *it;
		Vec3i face = meshes[primitive.first]->get_face(primitive.second);
		for (int i = 0; i < 3; i++) {
			Vec3f v = meshes[primitive.first]->get_vertice(face[i]);
			if (v[0] < min_corner[0])
				min_corner[0] = v[0];
			if (v[1] < min_corner[1])
				min_corner[1] = v[1];
			if (v[2] < min_corner[2])
				min_corner[2] = v[2];
			if (v[0] > max_corner[0])
				max_corner[0] = v[0];
			if (v[1] > max_corner[1])
				max_corner[1] = v[1];
			if (v[2] > max_corner[2])
				max_corner[2] = v[2];
		}
	}
	return AABB(min_corner, max_corner);
};

AABB AABB::get_bbox(const vector<Mesh*>& meshes) {
	vector<pair<int, int>> primitives;

	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < (meshes[i]->n_faces()); j++) {
			primitives.push_back(pair<int, int>(i, j));
		}
	}

	return get_bbox(meshes, primitives.begin(), primitives.end());
}

bool AABB::ray_intersection(const Ray& ray, Vec3f& entry, Vec3f& exit, float& t) const {
	// r.dir is unit direction vector of ray
	float dirfrac_x = 1.0f / (ray.get_direction()[0] + aabb_eps);
	float dirfrac_y = 1.0f / (ray.get_direction()[1] + aabb_eps);
	float dirfrac_z = 1.0f / (ray.get_direction()[2] + aabb_eps);
	// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
	// r.org is origin of ray
	float t1 = (min_corner[0] - ray.get_origin()[0]) * dirfrac_x;
	float t2 = (max_corner[0] - ray.get_origin()[0]) * dirfrac_x;
	float t3 = (min_corner[1] - ray.get_origin()[1]) * dirfrac_y;
	float t4 = (max_corner[1] - ray.get_origin()[1]) * dirfrac_y;
	float t5 = (min_corner[2] - ray.get_origin()[2]) * dirfrac_z;
	float t6 = (max_corner[2] - ray.get_origin()[2]) * dirfrac_z;

	float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
	float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

	// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
	if (tmax < 0)
	{
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
	{
		return false;
	}

	// The ray starts in the box
	if (tmin < 0) {
		t = tmax;
		entry = ray.get_origin();
		exit = entry + t * ray.get_direction();
		return true;
	}

	t = tmin;
	entry = ray.get_origin() + t * ray.get_direction();
	exit = ray.get_origin() + tmax * ray.get_direction();

	return true;
}

////https://stackoverflow.com/questions/17458562/efficient-aabb-triangle-intersection-in-c-sharp
//bool AABB::triangleIntersecton(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const
//{
//    double triangleMin, triangleMax;
//    double boxMin, boxMax;
//    
//    // Test the box normals (x-, y- and z-axes)
//    Vec3f boxNormals[3] = {
//        Vec3f(1,0,0),
//        Vec3f(0,1,0),
//        Vec3f(0,0,1)
//    };
//
//	vector<Vec3f> triangle_vertices = { p0, p1, p2 };
//
//    for (int i = 0; i < 3; i++)
//    {
//        Vec3f n = boxNormals[i];
//        project(triangle_vertices, boxNormals[i], triangleMin, triangleMax);
//        if (triangleMax < min_corner[i] || triangleMin > max_corner[i])
//            return false; // No intersection possible.
//    }
//
//	Vec3f triangle_normal = cross(p1 - p0, p2 - p0);
//
//	vector<Vec3f> bbox_vertices = getVertices();
//
//    // Test the triangle normal
//    double triangleOffset = dot(triangle_normal, p0);
//    project(bbox_vertices, triangle_normal, boxMin, boxMax);
//    if (boxMax < triangleOffset || boxMin > triangleOffset)
//        return false; // No intersection possible.
//
//    // Test the nine edge cross-products
//    Vec3f triangleEdges[3] = {
//        p0 - p1,
//        p1 - p2,
//        p2 - p0
//    };
//
//    for (int i = 0; i < 3; i++)
//        for (int j = 0; j < 3; j++)
//        {
//            // The box normals are the same as it's edge tangents
//            Vec3f axis = cross(triangleEdges[i], boxNormals[j]);
//            project(bbox_vertices, axis, boxMin, boxMax);
//            project(vector<Vec3f>{ p0, p1, p2 }, axis, triangleMin, triangleMax);
//            if (boxMax < triangleMin || boxMin > triangleMax)
//                return false; // No intersection possible
//        }
//    // No separating axis found.
//    return true;
//}
//




//float triangleMin, triangleMax;
//float boxMin, boxMax;
//Vec3f min_corner = center - extent;
//Vec3f max_corner = center + extent;
//
//// Test the box normals (x-, y- and z-axes)
//Vec3f boxNormals[3] = {
//	Vec3f(1,0,0),
//	Vec3f(0,1,0),
//	Vec3f(0,0,1)
//};
//
//vector<Vec3f> triangle_vertices = { p0, p1, p2 };
//
//for (int i = 0; i < 3; i++)
//{
//	Vec3f n = boxNormals[i];
//	project(triangle_vertices, boxNormals[i], triangleMin, triangleMax);
//	if (triangleMax < min_corner[i] || triangleMin > max_corner[i])
//		return false; // No intersection possible.
//}
//
//Vec3f triangle_normal = cross(p1 - p0, p2 - p0);
//
//vector<Vec3f> bbox_vertices = { min_corner, Vec3f(min_corner[0], min_corner[1], max_corner[2]),
//	Vec3f(min_corner[0], max_corner[1], min_corner[2]),
//Vec3f(min_corner[0], max_corner[1], max_corner[2]),
//Vec3f(max_corner[0], min_corner[1], min_corner[2]),
//Vec3f(max_corner[0], min_corner[1], max_corner[2]),
//Vec3f(max_corner[0], max_corner[1], min_corner[2]),
//max_corner };
//
//// Test the triangle normal
//double triangleOffset = dot(triangle_normal, p0);
//project(bbox_vertices, triangle_normal, boxMin, boxMax);
//if (boxMax < triangleOffset || boxMin > triangleOffset)
//return false; // No intersection possible.
//
//// Test the nine edge cross-products
//Vec3f triangleEdges[3] = {
//	p0 - p1,
//	p1 - p2,
//	p2 - p0
//};
//
//for (int i = 0; i < 3; i++)
//	for (int j = 0; j < 3; j++)
//	{
//		// The box normals are the same as it's edge tangents
//		Vec3f axis = cross(triangleEdges[i], boxNormals[j]);
//		project(bbox_vertices, axis, boxMin, boxMax);
//		project(vector<Vec3f>{ p0, p1, p2 }, axis, triangleMin, triangleMax);
//		if (boxMax < triangleMin || boxMin > triangleMax)
//			return false; // No intersection possible
//	}
//// No separating axis found.
//return true;

//bool SVDAG::triangleIntersection(const Vec3f& center, const Vec3f& extent, const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const {
//	float triangleMin = 0, triangleMax = 0;
//	float boxMin = 0, boxMax = 0;
//
//	// Test the box normals (x-, y- and z-axes)
//	Vec3f boxNormals[3] = {
//		Vec3f(1,0,0),
//		Vec3f(0,1,0),
//		Vec3f(0,0,1)
//	};
//
//	vector<Vec3f> triangle_vertices = { p0 - center, p1 - center, p2 - center };
//
//	for (int i = 0; i < 3; i++)
//	{
//		project(triangle_vertices, boxNormals[i], triangleMin, triangleMax);
//		if ((triangleMax < -extent[i]) || (triangleMin > extent[i]))
//			return false; // No intersection possible.
//	}
//
//	// Test the nine edge cross-products
//	Vec3f triangleEdges[3] = {
//		p0 - p1,
//		p1 - p2,
//		p2 - p0
//	};
//
//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//		{
//			// The box normals are the same as it's edge tangents
//			Vec3f axis = cross(triangleEdges[i], boxNormals[j]);
//			float r = project_extent(extent, axis);
//			project(triangle_vertices, axis, triangleMin, triangleMax);
//			if ((r < triangleMin) || (-r > triangleMax))
//				return false; // No intersection possible
//		}
//
//	Vec3f triangle_normal = cross(p1 - p0, p2 - p0);
//	triangle_normal = normalize(triangle_normal);
//
//	// Test the triangle normal
//	float triangleOffset = dot(triangle_normal, triangle_vertices[0]);
//	float r = project_extent(extent, triangle_normal);
//	if ((r < triangleOffset) || (-r > triangleOffset))
//		return false; // No intersection possible.
//
//	// No separating axis found.
//	return true;
//}