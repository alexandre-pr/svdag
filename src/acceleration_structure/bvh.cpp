#include "acceleration_structure/bvh.h"
#include <limits>
#include <queue>
#include <stack>

using namespace std;


BVH::BVH(const vector<Mesh*>& meshes, int leaf_size) {
	vector<pair<int, int>> primitives;

	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < (meshes[i]->n_faces()); j++) {
			primitives.push_back(pair<int, int>(i, j));
		}
	}

	int n_nodes = 1; // Number of nodes
	hierarchy.push_back(Node()); // The first node

	queue<PrimitiveGroup> queue; // A queue have a start element and an end element in the array or vertices
	// It is basically a fusion sort except the axis of sorting varies and nodes are added to the hierarchy at each step

	if (primitives.size() >= 1) {
		queue.push(PrimitiveGroup(0, 0, (int) primitives.size()));
	}

	while (!queue.empty()) {
		PrimitiveGroup slice = queue.front();
		assert(slice.begin < slice.end); // We shouldn't end up with empty slices
		queue.pop();

		// Slice with only one primitive: create leaf
		if (slice.end - slice.begin <= leaf_size) 
		{
			vector<pair<int, int>> p(primitives.begin() + slice.begin, primitives.begin() + slice.end);
			AABB bbox = AABB::get_bbox(meshes, p.begin(), p.end());
			hierarchy[slice.node_idx] = Node(p, bbox);
		}
		
		else {
			AABB bbox = AABB::get_bbox(meshes, primitives.begin() + slice.begin, primitives.begin() + slice.end);
			
			vector<float> size = { bbox.max_corner[0] - bbox.min_corner[0],
			bbox.max_corner[1] - bbox.min_corner[1],
			bbox.max_corner[2] - bbox.min_corner[2] };

			// We want to divide along the dimension of biggest size
			int mi = (int)(max_element(size.begin(), size.end()) - size.begin());
			auto sortRule = [mi, &meshes](const pair<int, int>& s1, const pair<int, int>& s2) {
				Vec3f b1 = meshes[s1.first]->get_barycenter(s1.second);
				Vec3f b2 = meshes[s2.first]->get_barycenter(s2.second);
				return b1[mi] < b2[mi];
			};
			sort(primitives.begin() + slice.begin, primitives.begin() + slice.end, sortRule);
			int median = (slice.begin + slice.end)/2;
			//float median_value = (get_barycenter(meshes, primitives[median].first, primitives[median].second)[mi]
			//	+ get_barycenter(meshes, primitives[median - 1].first, primitives[median - 1].second)[mi]) / 2;
			hierarchy.push_back(Node());
			hierarchy.push_back(Node());

			// The current node is finalized
			//hierarchy[slice.node_idx] = Node(n_nodes, n_nodes + 1, bbox, mi, median_value);
			hierarchy[slice.node_idx] = Node(n_nodes, n_nodes + 1, bbox);
			queue.push(PrimitiveGroup(n_nodes, slice.begin, median));
			queue.push(PrimitiveGroup(n_nodes+1, median, slice.end));
			n_nodes += 2; // Two new nodes have been added

		}
	}
};


