#include "scene/scene.h"

const Hit_BVH Scene::ray_intersection(const Ray& ray) const {
	Hit_BVH result = Hit_BVH();

	const Vec3f& ray_origin = ray.get_origin();
	std::vector<pair<float, int>> stack;
	int size = 0;
	double t = 0;
	int ll = (int)log(bvh.hierarchy.size());
	stack.reserve(ll);


	if (bvh.hierarchy.size() == 0)
		return result; // No hit

	// Nodes to check : in depth traversal, float is the negative entry distance to the node
	// because priority_queue returns biggest priority
	vector<pair<float, int>> kept;
	kept.reserve(ll);
	Vec3f entry, exit;
	float distance;

	if (bvh.hierarchy.size() == 1) {
		kept.push_back(pair<float, int>(0.0f, 0));
	}
	else {
		if (bvh.hierarchy[0].bbox.ray_intersection(ray, entry, exit, distance)) {
			stack.push_back(pair<float, int>(distance, bvh.hierarchy[0].left));
			stack.push_back(pair<float, int>(distance, bvh.hierarchy[0].right));
			size++; size++;
		}
	}

	while (size) {
		pair<float, int> dist_nodeidx = stack.back();
		stack.pop_back(); size--;
		//const Node& node = hierarchy[dist_nodeidx.second];

		if (bvh.hierarchy[dist_nodeidx.second].is_leaf) {
			kept.push_back(dist_nodeidx);
		}
		else {
			if (bvh.hierarchy[dist_nodeidx.second].bbox.ray_intersection(ray, entry, exit, distance)) {
				stack.push_back(pair<float, int>(distance, bvh.hierarchy[dist_nodeidx.second].left));
				stack.push_back(pair<float, int>(distance, bvh.hierarchy[dist_nodeidx.second].right));
				size++;size++;
			}

		}
	}

	float z_buffer = numeric_limits<float>::max();
	Hit hit;

	sort(kept.begin(), kept.end());
	for (const pair<float, int>& dist_nodeidx : kept) {
		if (dist_nodeidx.first >= z_buffer) {
			break;
		}
		const Node& node = bvh.hierarchy[dist_nodeidx.second];
		for (const pair<int, int>& primitive : node.primitives) {
			const Vec3i& f = meshes[primitive.first]->get_face(primitive.second);
			const Vec3f& p0 = meshes[primitive.first]->get_vertice(f[0]);
			const Vec3f& p1 = meshes[primitive.first]->get_vertice(f[1]);
			const Vec3f& p2 = meshes[primitive.first]->get_vertice(f[2]);
			if ((ray.intersectTriangle(p0, p1, p2, hit)) && (hit.distance < z_buffer)) {
				z_buffer = hit.distance;
				result = Hit_BVH(z_buffer, hit.b0, hit.b1, hit.b2, primitive.first, primitive.second);
			}
		}
	}
	//auto t02 = std::chrono::high_resolution_clock::now();
	//std::cout << t << std::endl<< std::chrono::duration_cast<std::chrono::nanoseconds>(t02 - t01).count()<<std::endl<<std::endl;
	return result;
}


const bool Scene::ray_blocked(const Ray& ray, float max_dist) const {
	if (bvh.hierarchy.size() == 0)
		return false; // No hit

	const Vec3f& ray_origin = ray.get_origin();
	std::vector<pair<float, int>> stack;
	int size = 0;
	double t = 0;
	int ll = (int)log(bvh.hierarchy.size());
	stack.reserve(ll);


	// Nodes to check : in depth traversal, float is the negative entry distance to the node
	// because priority_queue returns biggest priority
	vector<pair<float, int>> kept;
	kept.reserve(ll);
	Vec3f entry, exit;
	float distance;

	if (bvh.hierarchy.size() == 1) {
		kept.push_back(pair<float, int>(0.0f, 0));
	}
	else {
		if (bvh.hierarchy[0].bbox.ray_intersection(ray, entry, exit, distance)) {
			stack.push_back(pair<float, int>(distance, bvh.hierarchy[0].left));
			stack.push_back(pair<float, int>(distance, bvh.hierarchy[0].right));
			size++; size++;
		}
	}

	
	//auto t01 = std::chrono::high_resolution_clock::now();
	while (size) {

		pair<float, int> dist_nodeidx = stack.back();
		stack.pop_back(); size--;
		//const Node& node = hierarchy[dist_nodeidx.second];

		if (bvh.hierarchy[dist_nodeidx.second].is_leaf) {
			kept.push_back(dist_nodeidx);
		}
		else {
			if (bvh.hierarchy[dist_nodeidx.second].bbox.ray_intersection(ray, entry, exit, distance)) {
				stack.push_back(pair<float, int>(distance, bvh.hierarchy[dist_nodeidx.second].left));
				stack.push_back(pair<float, int>(distance, bvh.hierarchy[dist_nodeidx.second].right));
				size++;size++;
			}

		}
	}

	Hit hit;

	sort(kept.begin(), kept.end());
	for (const pair<float, int>& dist_nodeidx : kept) {
		const Node& node = bvh.hierarchy[dist_nodeidx.second];
		for (const pair<int, int>& primitive : node.primitives) {
			const Vec3i& f = meshes[primitive.first]->get_face(primitive.second);
			const Vec3f& p0 = meshes[primitive.first]->get_vertice(f[0]);
			const Vec3f& p1 = meshes[primitive.first]->get_vertice(f[1]);
			const Vec3f& p2 = meshes[primitive.first]->get_vertice(f[2]);
			if ((ray.intersectTriangle(p0, p1, p2, hit)) && (hit.distance > eps_scene) && (hit.distance < max_dist)) {
				return true;
			}
		}
	}
	//auto t02 = std::chrono::high_resolution_clock::now();
	//std::cout << t << std::endl<< std::chrono::duration_cast<std::chrono::nanoseconds>(t02 - t01).count()<<std::endl<<std::endl;

	return false;
}