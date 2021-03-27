#include <chrono>
#include <algorithm>
#include <map>
#include <set>
#include <numeric>
#include<stack>

#include "acceleration_structure/svdag.h"

void project(const vector<Vec3f>& points, const Vec3f& axis,
	float& min, float& max);
float project_extent(const Vec3f& extent, const Vec3f& axis);
Vec3i relativePos(const Vec3f& min_corner, const Vec3f& max_corner, Vec3f point);
void stackPush(stack<pair<uint, Vec3<bool>>>& stack, uint node_idx, Vec3<bool> child_pos, Vec3f& stride, AABB& voxel, int& depth);
void stackPop(stack<pair<uint, Vec3<bool>>>& stack, Vec3f& stride, AABB& voxel, int& depth);

SVDAG::SVDAG() : max_depth(0) {};

SVDAG::SVDAG(const vector<Mesh*>& meshes, const Vec3f& min_corner, const Vec3f& max_corner, size_t max_depth, bool verbose) :
	max_depth(max_depth), min_stride((max_corner - min_corner)/(float)pow(2,max_depth)), nodes(vector<vector<uint>>(max_depth)) , bbox(min_corner, max_corner)
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
	min_stride = (bbox.max_corner - bbox.min_corner) / (float)pow(2, max_depth);
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
	int depth = (int)(max_depth - 3);
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
	for (int depth = (int)(max_depth-4); depth >= 0; depth--) {
		
		size_t N_nodes = nodes_ptr[depth+1].size();
		

		map<uint,uint> indirection_table; // To keep track of original position after merging
		
		vector<uint> nodes_ptr_end(nodes_ptr[depth + 1].begin() + 1, nodes_ptr[depth + 1].end()); // Store the end of a node (pointer store the beginning)
		nodes_ptr_end.push_back((uint)nodes[depth + 1].size());

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
			new_nodes_ptr.push_back((uint)new_nodes.size()); // We update the node pointer
			for (uint j = nodes_ptr[depth + 1][indexes[i]]; j < nodes_ptr_end[indexes[i]]; j++) {
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



bool SVDAG::shadowRay(const Ray& ray, float t_max) const{
	int depth = 0;
	stack<pair<uint, Vec3<bool>>> stack; // A node and its relative position in its parent
	Vec3f stride = bbox.max_corner - bbox.min_corner;
	AABB voxel = AABB(bbox);
	const Vec3f& origin = ray.get_origin(); const Vec3f& direction = ray.get_direction();
	float t = 0; Vec3f exit_entry; int exit_direction = 0; uint child_idx;
	bool firstIteration=false;

	// Find the origin location (represented as a stack of uint, from root to last node)
	Vec3i relative_pos = relativePos(bbox.min_corner, bbox.max_corner, origin);

	// If the ray starts in the box, the stack must be filled.
	// It is important to ignore the "starting voxel", to avoid a primitive shadowing itself
	if ((relative_pos >= Vec3i(0,0,0)) && (relative_pos < Vec3i(2,2,2))) {// Ray origin inside the box
		stack.push(pair<uint, Vec3<bool>>(0, Vec3<bool>(0,0,0)));
		while (true) { // either exit because of max depth reached or because empty voxel
			uint node_idx = stack.top().first;
			if (node_idx == -1) // Empty node, we start the search from here (weird case, it shouldn't happen)
			{
				//cout << "Warning: Ray starting from empty voxel" << endl;
				break;
			}

			if (depth == max_depth - 2) { // Compressed levels
				firstIteration = true;
				break;
			}

			relative_pos = relativePos(voxel.min_corner, voxel.max_corner, origin);
			uchar child_no = relative_pos[0] * 4 + relative_pos[1] * 2 + relative_pos[2];
			getChild(depth, node_idx, child_no, child_idx);
			stackPush(stack, child_idx, Vec3<bool>(relative_pos[0], relative_pos[1], relative_pos[2]), stride, voxel, depth);
		}
	}

	else { // Ray origin outside the box
		//cout << "Warning: Ray starting outside the box" << endl;
		if (ray.intersectBox(bbox.min_corner, bbox.max_corner, t)) {
			exit_entry = origin + t * ray.get_direction();
			stack.push(pair<uint, Vec3<bool>>(0, Vec3<bool>(0, 0, 0)));
		}
		else
			return false;
	}

	while (!stack.empty()) {
		// Pop the last node
		pair<uint, Vec3<bool>> node = stack.top();
		uint node_idx = node.first;
		Vec3<bool> child_pos = node.second;

		if ((depth == max_depth - 2) || (node_idx == -1)) // Compressed levels or empty voxel
		{
			// The shift required for the ray to exit is computed in exit_direction (ADVANCE)
			if (node_idx == -1) {// Empty voxel
				ray.intersectBox(voxel.min_corner, voxel.max_corner, exit_direction, t, exit_entry);
			}
			else {
				bool blocked;
				if (firstIteration)
					blocked = shadowRayLeafInside(ray, leaves[node_idx], voxel.min_corner, voxel.max_corner, exit_entry, exit_direction, t);
				else
					blocked = shadowRayLeaf(ray, exit_entry, exit_direction, leaves[node_idx], voxel.min_corner, voxel.max_corner, exit_entry, exit_direction, t);
				if (blocked) // If minimal size and filled: return true
				{
					return (t < t_max);
				}
			}

			// If not intercepted
			// Pop nodes until you can satisfy the shift (POP)
			bool target = (direction[exit_direction] > 0) ? 1 : 0; // We want to shift to 1 if direction>0
			while (child_pos[exit_direction] == target) { // We want to increment child pos on the right direction
				stackPop(stack, stride, voxel, depth);
				if (stack.empty())
					return false; // We exited the octree
				node = stack.top();
				node_idx = node.first;
				child_pos = node.second;
			}
			child_pos[exit_direction] = target;
			// Pop the node that can be changed
			stackPop(stack, stride, voxel, depth);

			if (stack.empty())
				return false; // We exited the octree
			uint parent_node_idx = stack.top().first;
			getChild(depth, parent_node_idx, 4 *child_pos[0] + 2 * child_pos[1] + child_pos[2], child_idx);
			stackPush(stack, child_idx, child_pos, stride, voxel, depth);
		}

		else {
			// Else
			// Find the child that is at the right location (PUSH)
			Vec3<bool> new_child_pos;

			for (int i = 0; i < 3; i++) {
				if (i == exit_direction) {
					if (direction[i] < 0)
						new_child_pos[i] = 1;
				}
				else {
					new_child_pos[i] = min((int)(2 * (exit_entry[i] - voxel.min_corner[i]) / (voxel.max_corner[i] - voxel.min_corner[i])),1);
				}
			}
			getChild(depth, node_idx, 4 * new_child_pos[0] + 2 * new_child_pos[1] + new_child_pos[2], child_idx);
			stackPush(stack, child_idx, new_child_pos, stride, voxel, depth);
		}
		
		if (t > t_max)
			return false; // Too far

		firstIteration = false;
	}

	cout << "Warning: Unusual exit" << endl;
	return false;
}

// Gives which child bbox the point is in ({0,1}, {0,1}, {0,1})
Vec3i relativePos(const Vec3f& min_corner, const Vec3f& max_corner, Vec3f point) {
	Vec3f rel_pos = (point - min_corner) * 2 / (max_corner - min_corner);
	return Vec3i((int)rel_pos[0], (int)rel_pos[1], (int)rel_pos[2]);
}

void stackPush(stack<pair<uint,Vec3<bool>>>& stack, uint node_idx, Vec3<bool> child_pos, Vec3f& stride, AABB& voxel, int& depth) {
	stack.push(pair<uint, Vec3<bool>>(node_idx, child_pos));
	depth++;
	stride /= 2;
	Vec3f offset = stride * Vec3f(child_pos[0], child_pos[1], child_pos[2]);
	Vec3f m_corner = voxel.min_corner + offset;
	voxel = AABB(m_corner, m_corner+stride);
}

void stackPop(stack<pair<uint, Vec3<bool>>>& stack, Vec3f& stride, AABB& voxel, int& depth) {
	Vec3<bool> child_pos = stack.top().second;
	stack.pop(); // Pop the node
	// Updating parameters
	Vec3f offset = stride * Vec3f(child_pos[0], child_pos[1], child_pos[2]);
	Vec3f m_corner = voxel.min_corner - offset;
	depth--;
	stride *= 2;
	voxel = AABB(m_corner, m_corner + stride);
}

//////////////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////////////

bool SVDAG::shadowRayLeaf(const Ray& ray, const Vec3f& entry_point, int entry_direction, uint64_t leaf, const Vec3f& min_corner, 
	const Vec3f& max_corner, Vec3f& exit, int& exit_direction, float& t) const{
	// Leaf traversal

	const Vec3f& direction = ray.get_direction();
	Vec3i voxel_pos; // Position of the child in the 4*4*4 cube, [[0,4[[^3
	Vec3f stride = (max_corner - min_corner) / 4;

	for (int i = 0; i < 3; i++) {
		if (i == entry_direction) {
			if (direction[i] < 0)
				voxel_pos[i] = 3;
		}
		else {
			voxel_pos[i] = min((int)(4 * (entry_point[i] - min_corner[i]) / (max_corner[i] - min_corner[i])), 3);
		}
	}

	while (true) {
		if (readLeaf(leaf, voxel_pos[0], voxel_pos[1], voxel_pos[2])) // Voxel is full
			return true;
		else {
			Vec3f m_corner = min_corner + stride * Vec3f((float)voxel_pos[0], (float)voxel_pos[1], (float)voxel_pos[2]);
			ray.intersectBox(m_corner, m_corner + stride * Vec3f(1, 1, 1), exit_direction, t, exit);
			if (direction[exit_direction] > 0) {
				voxel_pos[exit_direction]++;
				if (voxel_pos[exit_direction] == 4)
					return false;
			}
			else {
				voxel_pos[exit_direction]--;
				if (voxel_pos[exit_direction] == -1)
					return false;
			}
		}
	}
}

bool SVDAG::shadowRayLeafInside(const Ray& ray, uint64_t leaf, const Vec3f& min_corner, const Vec3f& max_corner,
	Vec3f& exit, int& exit_direction, float& t) const{
	// To be called when the ray starts from inside a leaf
	Vec3f a =  (ray.get_origin() - min_corner) * 4/ (max_corner - min_corner);
	Vec3i voxel_pos;
	for (int i = 0; i < 3; i++) {
		voxel_pos[i] = min((int)a[i], 3);
	}
	Vec3f stride = (max_corner - min_corner) / 4;
	Vec3f direction = ray.get_direction();

	bool isOriginVoxel = true;

	while (true) {
		if (readLeaf(leaf, voxel_pos[0], voxel_pos[1], voxel_pos[2]) && (!isOriginVoxel)) // Voxel is full except the origin voxel
			return true;
		else {
			Vec3f m_corner = min_corner + stride * Vec3f((float)voxel_pos[0], (float)voxel_pos[1], (float)voxel_pos[2]);
			ray.intersectBox(m_corner, m_corner + stride * Vec3f(1, 1, 1), exit_direction, t, exit);
			if (direction[exit_direction] > 0) {
				voxel_pos[exit_direction]++;
				if (voxel_pos[exit_direction] == 4)
					return false;
			}
			else {
				voxel_pos[exit_direction]--;
				if (voxel_pos[exit_direction] == -1)
					return false;
			}
		}
		isOriginVoxel = false;
	}
}

bool SVDAG::readLeaf(uint64_t leaf, int pos_x, int pos_y, int pos_z) const{ // Pos in  [[0,4[[^3
	bool i = (bool)(pos_x / 2);
	bool j = (bool)(pos_y / 2);
	bool k = (bool)(pos_z / 2);
	bool ic = (bool)(pos_x - 2 * i);
	bool jc = (bool)(pos_y - 2 * j);
	bool kc = (bool)(pos_z - 2 * k);
	leaf = leaf >> ((1 - i) * 4 + (1 - j) * 2 + (1 - k)) * 8;
	return get_bit(leaf, ic * 4 + 2 * jc + kc);
}

void SVDAG::getChild(int depth, uint node_idx, uchar child_no, uint& child_idx) const {

	uchar childmask = (uchar)nodes[depth][node_idx];
	if (get_bit(childmask, child_no)) {
		int ptr_offset = 1;
		for (uchar i = 0; i < child_no; i++) { // Finding the position of the pointer
			if (get_bit(childmask, i))
				ptr_offset++;
		}
		child_idx = nodes[depth][node_idx + ptr_offset];
	}
	else
		child_idx = -1; // -1 means numeric_limit<uint>::max() here, which normally shouldn't be used for anything else
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