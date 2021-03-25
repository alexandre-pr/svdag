#include "svdag.h"
#include <chrono>
#include <algorithm>
#include <map>
#include <set>
#include <numeric>

void project(const vector<Vec3f>& points, const Vec3f& axis,
	float& min, float& max);
float project_extent(const Vec3f& extent, const Vec3f& axis);

SVDAG::SVDAG() : max_depth(0) {};

SVDAG::SVDAG(const vector<Mesh*>& meshes, const Vec3f& min_corner, const Vec3f& max_corner, size_t max_depth, bool verbose) :
	max_depth(max_depth), nodes(vector<vector<uint>>(max_depth)) , bbox(min_corner, max_corner)
{
	computeSVDAG(meshes, verbose);
}

SVDAG::SVDAG(const vector<Mesh*>& meshes, size_t max_depth, bool verbose):
	max_depth(max_depth), nodes(vector<vector<uint>>(max_depth))
{
	vector<pair<int, int>> primitives;

	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < (meshes[i]->n_faces()); j++) {
			primitives.push_back(pair<int, int>(i, j));
		}
	}

	// Find the min/max corners
	bbox = AABB::get_bbox(meshes, primitives.begin(), primitives.end());
	computeSVDAG(meshes, verbose);
}

void SVDAG::computeSVDAG(const vector<Mesh*>& meshes, bool verbose) {
	assert(max_depth >= 3);
	vector<vector<uint>> nodes_ptr(max_depth - 2);
	// Stores the pointer to the beginning of the nodes, 
	//we don't need the two last levels because they will be compressed

	// Construction the SVO
	if (verbose)
		std::cout << "Begin SVO construction ..." << std::endl;


	vector<pair<int, int>> primitives;

	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < (meshes[i]->n_faces()); j++) {
			primitives.push_back(pair<int, int>(i, j));
		}
	}

	if (max_depth > 0)
	{
		if (primitives.size()) {
			Vec3f half_diagonal = (bbox.max_corner - bbox.min_corner) / 2;
			computeSVO(meshes, bbox.min_corner, half_diagonal, primitives, 1, nodes_ptr);
		}
		else {
			nodes[0].push_back(0);
			n_nodes++;
		}
	}

	if (verbose) {
		size_t size = 0;
		for (size_t i = 0; i < nodes.size(); i++) {
			size += nodes[i].size() * 4;
		}

		std::cout << "size: " << size << std::endl;
		std::cout << "nodes: " << n_nodes << std::endl;
	}

	compressSVO(nodes_ptr, verbose);

	computeDAG(nodes_ptr, verbose);
}



uint SVDAG::computeSVO(const vector<Mesh*>& meshes, const Vec3f& min_corner,
	const Vec3f& half_diagonal, const std::vector<pair<int, int>>& primitives,
	size_t depth, vector<vector<uint>>& nodes_ptr)
{
	Vec3f half_diagonal_child = half_diagonal / 2;
	std::vector<uint> node = { 0 };

	// i * 4 + 2 * j + k -> child_no
	for (bool i : {0, 1}) {
		for (bool j : {0, 1}) {
			for (bool k : {0, 1}) {
				Vec3f min_corner_child = min_corner + Vec3f(i, j, k) * half_diagonal;
				Vec3f center = min_corner_child + half_diagonal_child;
				//AABB bbox_child = AABB(min_corner_child, min_corner_child + half_diagonal);
				vector<pair<int, int>> primitives_child;
				//primitives_child.reserve(primitives.size());
				for (int i = 0; i < primitives.size(); i++) {
					const Mesh* mesh = meshes[primitives[i].first];
					const Vec3i& f = mesh->get_face(primitives[i].second);
					bool intersect = triangleIntersection(center, half_diagonal_child, mesh->get_vertice(f[0]),
						mesh->get_vertice(f[1]),
						mesh->get_vertice(f[2]));
					if (intersect) {
						primitives_child.push_back(primitives[i]);
					}
				}

				if (primitives_child.size()) { // There is an intersection: create the node
					node[0] = set_bit(node[0], i * 4 + 2 * j + k);
					if (max_depth > depth) {
						uint child_ptr = computeSVO(meshes, min_corner_child, half_diagonal_child,
							primitives_child, depth + 1, nodes_ptr);
						node.push_back(child_ptr);
					}
				}
			}
		}
	}
	uint node_ptr = (uint)nodes[depth - 1].size();
	for (uint d : node) {
		nodes[depth - 1].push_back(d);
		n_nodes++;
	}

	if (depth < max_depth - 1)
		nodes_ptr[depth - 1].push_back(node_ptr); //Position of the node
	return node_ptr;
};

void SVDAG::compressSVO(const vector<vector<uint>>& nodes_ptr, bool verbose) {
	// Compressing the lower levels of the SVO into leaves
	if (verbose)
		std::cout << "Compressing lower levels..."<<std::endl;

	for (uint node_ptr : nodes_ptr[max_depth - 3]) { // These nodes should point towards uint64 leaves
		int ptr_offset = 1;
		uchar childmask = nodes[max_depth - 3][node_ptr];
		for (uchar i = 0; i < 8; i++) {

			if (get_bit(childmask, i)) { // If child (= leaf): create the leaf
				// Creating leaf
				uint64_t leaf = 0;
				int child_ptr_offset = 1;
				uchar child_childmask = nodes[max_depth - 2][nodes[max_depth - 3][node_ptr + ptr_offset]];
				for (uchar j = 0; j < 8; j++) {
					leaf = leaf << 8;
					if (get_bit(child_childmask, j)) {
						uchar grandchild_childmask = nodes[max_depth - 1][nodes[max_depth - 2][nodes[max_depth - 3][node_ptr + ptr_offset] + child_ptr_offset]];
						leaf += grandchild_childmask;
						child_ptr_offset++;
					}
				}

				// Updating pointer
				nodes[max_depth - 3][node_ptr + ptr_offset] = (uint)leaves.size();
				leaves.push_back(leaf);
				ptr_offset++;
			};
		}
	}

	// Removing the depths that are now leaves
	nodes.pop_back();
	nodes.pop_back();

	if (verbose) {
		size_t size = 0;
		for (size_t i = 0; i < nodes.size(); i++) {
			size += nodes[i].size() * 4;
		}

		size_t n = 0;
		for (size_t i = 0; i < nodes_ptr.size(); i++) {
			n += nodes_ptr[i].size();
		}

		std::cout << "size: " << size + 8 * leaves.size() << std::endl;
		std::cout << "nodes: " << n << std::endl;
		std::cout << "leaves: " << leaves.size() << std::endl;
	}

}

void SVDAG::computeDAG(vector<vector<uint>>& nodes_ptr, bool verbose) {
	if (verbose)
		std::cout << "Create DAG..."<<std::endl;
	

	// Merging leaves

	// O(n log (k)) where k is the number of different leaves,
	set<uint64_t> set_leaves;
	for (uint64_t leaf : leaves) {
		if (set_leaves.find(leaf) == set_leaves.end()) {
			set_leaves.insert(leaf);
		}
	}
	vector<uint64_t> kept_leaves(set_leaves.begin(), set_leaves.end());
	set_leaves.clear();
	
	// Update the nodes of the last level accordingly
	int depth = max_depth - 3;
	size_t N_nodes = nodes_ptr[depth].size();
	for (uint i = 0; i < N_nodes - 1; i++) { // Points towards the child masks
		for (uint j = nodes_ptr[depth][i] + 1; j < nodes_ptr[depth][i + 1]; j++) {
			size_t new_leaf_idx = equal_range(kept_leaves.begin(), kept_leaves.end(), leaves[nodes[depth][j]]).first - kept_leaves.begin();
			nodes[depth][j] = (uint)new_leaf_idx;
		}
	}
	for (uint j = nodes_ptr[depth][N_nodes - 1]+1; j < nodes[depth].size(); j++) {
		size_t new_leaf_idx = equal_range(kept_leaves.begin(), kept_leaves.end(), leaves[nodes[depth][j]]).first - kept_leaves.begin();
		nodes[depth][j] = (uint)new_leaf_idx;
	}

	leaves = kept_leaves;


	// Doing the same for nodes
	for (int depth = max_depth-4; depth >= 0; depth--) {
		
		size_t N_nodes = nodes_ptr[depth+1].size();
		

		map<uint,uint> indirection_table; // To keep track of original position after merging
		
		vector<uint> nodes_ptr_end(nodes_ptr[depth + 1].begin() + 1, nodes_ptr[depth + 1].end()); // Store the end of a node (pointer store the beginning)
		nodes_ptr_end.push_back(nodes[depth + 1].size());

		// Sorting the nodes
		auto compareNodes = [&] (uint i1, uint i2) {
			vector<uint> node1(nodes[depth + 1].begin() + nodes_ptr[depth + 1][i1], 
				nodes[depth + 1].begin() + nodes_ptr_end[i1]);
			vector<uint> node2(nodes[depth + 1].begin() + nodes_ptr[depth + 1][i2],
				nodes[depth + 1].begin() + nodes_ptr_end[i2]);
			return node1<node2; // Lexicographic order
		};

		vector<uint> indexes(N_nodes); // Since we must sort both nodes_ptr[depth+1] and nodes_ptr_end at the same time, we use this vector
		iota(indexes.begin(), indexes.end(), 0);
		sort(indexes.begin(), indexes.end(), compareNodes);

		
		// Removing duplicates nodes in the nodes_ptr/nodes_ptr_end representation
		auto equalityNodes = [&](uint i1, uint i2) {
			vector<uint> node1(nodes[depth + 1].begin() + nodes_ptr[depth + 1][i1],
				nodes[depth + 1].begin() + nodes_ptr_end[i1]);
			vector<uint> node2(nodes[depth + 1].begin() + nodes_ptr[depth + 1][i2],
				nodes[depth + 1].begin() + nodes_ptr_end[i2]);
			return node1 == node2; // Lexicographic order
		};
		assert(N_nodes > 0);
		int index = 0; // By default there is at least the first value
		for (int i = 0; i < N_nodes; i++)
		{
			// We test the equality of the nodes
			if (equalityNodes(indexes[index], indexes[i]))
			{
				// We store a map from the old childmask position to the index of the corresponding node in the compressed list (we don't know yet at what position this node will be)
				indirection_table.insert(pair<uint,uint>(nodes_ptr[depth + 1][indexes[i]], index)); 
				continue;
			}

			// New value, we stored it after the already found unique values
			index++;
			indexes[index] = indexes[i];
			indirection_table.insert(pair<uint, uint>(nodes_ptr[depth + 1][indexes[i]], index));
		}
		// Kept nodes:
		indexes.resize(index+1); 
		
		
		// Removing duplicate in the storage
		vector<uint> new_nodes;
		vector<uint> new_nodes_ptr;
		for (int i = 0; i < indexes.size(); i++) {
			new_nodes_ptr.push_back(new_nodes.size()); // We update the node pointer
			for (int j = nodes_ptr[depth + 1][indexes[i]]; j < nodes_ptr_end[indexes[i]]; j++) {
				new_nodes.push_back(nodes[depth + 1][j]);
			}
		}
		nodes_ptr[depth + 1] = new_nodes_ptr; // Updated version of node ptr

		nodes[depth + 1] = new_nodes;

		//Updating upper nodes
		N_nodes = nodes_ptr[depth].size();
		for (uint i = 0; i < N_nodes - 1; i++) {
			for (uint j = nodes_ptr[depth][i] + 1; j < nodes_ptr[depth][i + 1]; j++) {
				uint old_node_idx = nodes[depth][j];
				nodes[depth][j] = nodes_ptr[depth+1][indirection_table[old_node_idx]];
			}
		}
		for (int j = nodes_ptr[depth][N_nodes - 1]+1; j < nodes[depth].size(); j++) {
			uint old_node_idx = nodes[depth][j];
			nodes[depth][j] = nodes_ptr[depth+1][indirection_table[old_node_idx]];
		}
	}


	if (verbose) {
		size_t size = 0;
		for (size_t i = 0; i < nodes.size(); i++) {
			size += nodes[i].size() * 4;
		}

		size_t n = 0;
		for (size_t i = 0; i < nodes_ptr.size(); i++) {
			n += nodes_ptr[i].size();
		}

		std::cout << "size: " << size + 8 * leaves.size() << std::endl;
		std::cout << "nodes: " << n << std::endl;
		std::cout << "leaves: " << leaves.size() << std::endl;
	}


}


//https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
//Smart implementation that avoids redundant computations
bool SVDAG::triangleIntersection(const Vec3f& center, const Vec3f& extent, const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const {
	float triangleMin = 0, triangleMax = 0;
	float boxMin = 0, boxMax = 0;

	// Test the box normals (x-, y- and z-axes)
	Vec3f boxNormals[3] = {
		Vec3f(1,0,0),
		Vec3f(0,1,0),
		Vec3f(0,0,1)
	};

	vector<Vec3f> triangle_vertices = { p0 - center, p1 - center, p2 - center };

	for (int i = 0; i < 3; i++)
	{
		Vec3f n = boxNormals[i];
		project(triangle_vertices, boxNormals[i], triangleMin, triangleMax);
		if (triangleMax < -extent[i] || triangleMin  > extent[i])
			return false; // No intersection possible.
	}

	// Test the nine edge cross-products
	Vec3f triangleEdges[3] = {
		p0 - p1,
		p1 - p2,
		p2 - p0
	};

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			// The box normals are the same as it's edge tangents
			Vec3f axis = cross(triangleEdges[i], boxNormals[j]);
			float r = project_extent(extent, axis);
			project(triangle_vertices, axis, triangleMin, triangleMax);
			if (r < triangleMin || -r > triangleMax)
				return false; // No intersection possible
		}

	Vec3f triangle_normal = cross(p1 - p0, p2 - p0);

	// Test the triangle normal
	float triangleOffset = dot(triangle_normal, triangle_vertices[0]);
	float r = project_extent(extent, triangle_normal);
	if (r < triangleOffset || -r> triangleOffset)
		return false; // No intersection possible.

	// No separating axis found.
	return true;
}

void project(const vector<Vec3f>& points, const Vec3f& axis,
	float& min, float& max)
{
	min = numeric_limits<float>::max();
	max = numeric_limits<float>::min();
	for (const Vec3f& p : points)
	{
		float val = dot(axis, p);
		if (val < min) min = val;
		if (val > max) max = val;
	}
}

float project_extent(const Vec3f& extent, const Vec3f& axis) {
	return extent[0] * abs(dot(Vec3f(1, 0, 0), axis)) +
		extent[1] * abs(dot(Vec3f(0, 1, 0), axis)) +
		extent[2] * abs(dot(Vec3f(0, 0, 1), axis));
}