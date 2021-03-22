#include "svo.h"

SVO::SVO() : max_depth(0) {};

SVO::SVO(const vector<Mesh*>& meshes, size_t max_depth, bool verbose) : 
	max_depth(max_depth), nodes(vector<vector<uint>>(max_depth))

{
	if (verbose)
		std::cout << "Begin SVO construction ...";

	vector<pair<int, int>> primitives;

	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < (meshes[i]->n_faces()); j++) {
			primitives.push_back(pair<int, int>(i, j));
		}
	}

	// Find the min/max corners
	bbox = AABB::get_bbox(meshes, primitives.begin(), primitives.end());

	if (max_depth > 0)
	{
		if (primitives.size()) {
			Vec3f half_diagonal = (bbox.max_corner - bbox.min_corner) / 2;
			computeSVO(meshes, bbox.min_corner, half_diagonal, primitives, 1);
		}
		else {
			nodes[0].push_back(0);
			n_nodes++;
		}
	}
	else {
		leaves.push_back(primitives.size());
		n_leaves++;
	}

	if (verbose) {
		std::cout << "done" << std::endl;

		size_t size = 0;
		for (size_t i = 0; i < nodes.size(); i++) {
			size += nodes[i].size() * 4;
		}

		std::cout << "size: " << size + n_leaves << std::endl;
		std::cout << "nodes: " << n_nodes << std::endl;
		std::cout << "leaves: " << n_leaves << std::endl;
	}
}

uint SVO::computeSVO(const vector<Mesh*>& meshes, const Vec3f& min_corner, const Vec3f& half_diagonal, const std::vector<pair<int, int>>& primitives, size_t depth) {
	Vec3f half_diagonal_child = half_diagonal / 2;
	std::vector<uint> node = { 0 };

	// i + 2 * j + 4 * k -> child_no
	for (bool i : {0, 1}) {
		for (bool j : {0, 1}) {
			for (bool k : {0, 1}) {
				Vec3f min_corner_child = min_corner + Vec3f(i, j, k) * half_diagonal;
				AABB bbox_child = AABB(min_corner_child, min_corner_child + half_diagonal);

				vector<pair<int, int>> primitives_child;
				for (int i = 0; i < primitives.size(); i++) {
					const Vec3i& f = meshes[primitives[i].first]->get_face(primitives[i].second);

					if (bbox_child.triangleIntersecton(meshes[primitives[i].first]->get_vertice(f[0]),
						meshes[primitives[i].first]->get_vertice(f[1]),
						meshes[primitives[i].first]->get_vertice(f[2]))) {
						primitives_child.push_back(primitives[i]);
					}
				}
				
				if (primitives_child.size()) { // There is an intersection: create the node
					node[0] = set_bit(node[0], i + 2 * j + 4 * k);
					if (max_depth > depth) {
						uint child_ptr = computeSVO(meshes, min_corner_child, half_diagonal_child, primitives_child, depth + 1);
						node.push_back(child_ptr);
					}
					else { // Create a leaf
						node.push_back(leaves.size());
						leaves.push_back(1);
						n_leaves++;
					}
				}
			}
		}
	}
	uint node_ptr = nodes[depth-1].size();
	for (uint d : node) {
		nodes[depth-1].push_back(d);
		n_nodes++;
	}
	return node_ptr;
};