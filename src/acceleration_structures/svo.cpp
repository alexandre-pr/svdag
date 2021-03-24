#include "svo.h"
#include <chrono>

void project(const vector<Vec3f>& points, const Vec3f& axis,
	double& min, double& max);
float project_extent(const Vec3f& extent, const Vec3f& axis);

SVO::SVO() : max_depth(0) {};

SVO::SVO(const vector<Mesh*>& meshes, size_t max_depth, bool verbose) : 
	max_depth(max_depth), nodes(vector<vector<uint>>(max_depth))

{
	assert(max_depth >= 3);
	vector<vector<uint>> nodes_ptr(max_depth-2); 
	// Stores the pointer to the beginning of the nodes, 
	//we don't need the two last levels because they will be compressed

	// Construction the SVO
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

	//computeDAG(nodes_ptr, verbose);

}

uint SVO::computeSVO(const vector<Mesh*>& meshes, const Vec3f& min_corner, 
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
	uint node_ptr = (uint)nodes[depth-1].size();
	for (uint d : node) {
		nodes[depth-1].push_back(d);
		n_nodes++;
	}

	if (depth < max_depth - 1)
		nodes_ptr[depth - 1].push_back(node_ptr); //Position of the node
	return node_ptr;
};

void SVO::compressSVO(const vector<vector<uint>>& nodes_ptr, bool verbose) {
	// Compressing the lower levels of the SVO into leaves
	if (verbose)
		std::cout << "Compressing lower levels...";

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
						uchar grandchild_childmask = nodes[max_depth-1][nodes[max_depth - 2][nodes[max_depth - 3][node_ptr + ptr_offset] + child_ptr_offset]];
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


//https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
//Smart implementation that avoids redundant computations
bool SVO::triangleIntersection(const Vec3f& center, const Vec3f& extent, const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const {
	double triangleMin=0, triangleMax=0;
	double boxMin=0, boxMax=0;

	// Test the box normals (x-, y- and z-axes)
	Vec3f boxNormals[3] = {
		Vec3f(1,0,0),
		Vec3f(0,1,0),
		Vec3f(0,0,1)
	};

	vector<Vec3f> triangle_vertices = { p0 - center, p1 - center, p2 - center};

	for (int i = 0; i < 3; i++)
	{
		Vec3f n = boxNormals[i];
		project(triangle_vertices, boxNormals[i], triangleMin, triangleMax);
		if (triangleMax < -extent[i] || triangleMin > extent[i])
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
	double triangleOffset = dot(triangle_normal, triangle_vertices[0]);
	float r = project_extent(extent, triangle_normal);
	if (r < triangleOffset || -r > triangleOffset)
		return false; // No intersection possible.
	
	// No separating axis found.
	return true;
}

void project(const vector<Vec3f>& points, const Vec3f& axis,
	double& min, double& max)
{
	min = numeric_limits<double>::max();
	max = numeric_limits<double>::min();
	for (const Vec3f& p : points)
	{
		double val = dot(axis, p);
		if (val < min) min = val;
		if (val > max) max = val;
	}
}

float project_extent(const Vec3f& extent, const Vec3f& axis) {
	return extent[0] * abs(dot(Vec3f(1,0,0), axis)) +
		extent[1] * abs(dot(Vec3f(0, 1, 0), axis))  +
		extent[2] * abs(dot(Vec3f(0, 0, 1), axis));
}