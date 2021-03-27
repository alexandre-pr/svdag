#include "scene/mesh.h"
#include "acceleration_structure/aabb.h"
#include <utility>
#include <algorithm>

// BVH is a vector of nodes

// Node index and slice of the primitive list that is used
struct PrimitiveGroup {
	int node_idx;
	int begin;
	int end; // Excluded

	PrimitiveGroup(int node_idx, int begin, int end) : 
		node_idx(node_idx), begin(begin), end(end) {};
};

struct Node {
	// Either a leaf
	bool is_leaf;
	
	std::vector<std::pair<int, int>> primitives;
	
	// Or a node
	AABB bbox;

	//int dimension; // Dimension along which the division was made
	//float median; // Median of the division
	
	int left;
	int right;

	Node() {}; // Placeholder before actual node is computed

	Node(std::vector<std::pair<int, int>> primitives, AABB bbox) : bbox(bbox), primitives(primitives), is_leaf(true) {};

	//Node(int left, int right, AABB bbox, int dimension, float median) : left(left), right(right), bbox(bbox), is_leaf(false), 
	//	dimension(dimension), median(median) {};

	Node(int left, int right, AABB bbox) : left(left), right(right), bbox(bbox), 
		is_leaf(false) {};

};

struct Hit_BVH: public Hit{
	int primitive_mesh_idx;
	int primitive_idx;

	Hit_BVH() : Hit() {};
	Hit_BVH(float distance, float b0, float b1, float b2, int mesh_idx, int idx) :
		Hit(distance, b0, b1, b2), primitive_mesh_idx(mesh_idx), primitive_idx(idx) {};
};


class BVH {
public:
	BVH() {}

	BVH(const std::vector<Mesh*>& meshes, int leaf_size=1);

	// Returns all the primitives that could intersect the ray
	//std::vector<std::pair<int, int>> ray_intersection(const Ray& ray) const;


	std::vector<Node> hierarchy;
	float epsilon = 0.000001f;
};