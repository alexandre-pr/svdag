#pragma once

#include "helper/bit_operations.h"
#include <vector>
#include <list>
#include "math/vec3.h"
#include "geometry/ray.h"
#include "scene/mesh.h"
#include "acceleration_structure/aabb.h"

using namespace std;
using uint = uint32_t;
using uchar = unsigned char;

static float svdag_eps = 1e-10f;

// On my computer, an unsigned int is 32bits so I used this format

class SVDAG {
public:
    /*inline vector<uint>& operator [](uint i) {
        return nodes[i];
    }

    inline const vector<uint>& operator [](uint i) const {
        return nodes[i];
    }*/


    // Constructing the data structure
    SVDAG();

    SVDAG(const vector<Mesh*>& meshes, size_t max_depth = 8, bool verbose=true);

    SVDAG(const vector<Mesh*>& meshes, const Vec3f& min_corner, const Vec3f& max_corner, size_t max_depth = 8, bool verbose = true);

    void computeSVDAG(const vector<Mesh*>& meshes, bool verbose);

    uint computeSVO(const vector<Mesh*>& meshes, const Vec3f& min_corner, const Vec3f& half_diagonal, const std::vector<pair<int, int>>& primitives, size_t depth, vector<vector<uint>>& nodes_ptr); 
    // Return the index of the added node (in the depth-th vector of data)

    void compressSVO(const vector<vector<uint>>& nodes_ptr, bool verbose);
    void computeDAG(vector<vector<uint>>& nodes_ptr, bool verbose);

    bool triangleIntersection(const Vec3f& center, const Vec3f& extent, const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) const;

    bool readLeaf(uint64_t leaf, int pos_x, int pos_y, int pos_z) const;
    void getChild(int depth, uint node_idx, uchar child_no, uint& child_idx) const;

    // Shading
    bool shadowRay(const Ray& ray, float t_max) const;
    bool shadowRayLeaf(const Ray& ray, const Vec3f& entry_point, int entry_direction, uint64_t leaf, const Vec3f& min_corner, const Vec3f& max_corner, Vec3f& exit, int& exit_direction, float& t) const;
    bool shadowRayLeafInside(const Ray& ray, uint64_t leaf, const Vec3f& min_corner, const Vec3f& max_corner, Vec3f& exit, int& exit_direction, float& t) const;

    AABB bbox; // Main bbox: defines the main corners
    size_t max_depth;
    vector<vector<uint>> nodes;
    vector<uint64_t> leaves;
    Vec3f min_stride;

    size_t n_nodes=0;
};