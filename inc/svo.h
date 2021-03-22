#pragma once

#include "bit_operations.h"
#include <vector>
#include <list>
#include "vec3.h"
#include "ray.h"
#include "mesh.h"
#include "aabb.h"

using namespace std;
using uint = uint32_t;
using uchar = unsigned char;

// On my computer, an unsigned int is 32bits so I used this format

class SVO {
public:
    /*inline vector<uint>& operator [](uint i) {
        return nodes[i];
    }

    inline const vector<uint>& operator [](uint i) const {
        return nodes[i];
    }*/

    SVO();

    SVO(const vector<Mesh*>& meshes, size_t max_depth = 8, bool verbose=true);

    uint computeSVO(const vector<Mesh*>& meshes, const Vec3f& min_corner, const Vec3f& half_diagonal, const std::vector<pair<int, int>>& primitives, size_t depth, vector<vector<uint>>& nodes_ptr); 
    // Return the index of the added node (in the depth-th vector of data)

    void compressSVO(const vector<vector<uint>>& nodes_ptr, bool verbose);

    AABB bbox; // Main bbox: defines the main corners
    int max_depth;
    vector<vector<uint>> nodes;
    vector<uint64_t> leaves;

    size_t n_nodes=0;
};