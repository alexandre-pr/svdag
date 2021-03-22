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
    inline vector<uint>& operator [](uint i) {
        return data[i];
    }

    inline const vector<uint>& operator [](uint i) const {
        return data[i];
    }

    SVO(int depth, const Vec3f& min_corner, const Vec3f& max_corner);

    void compute_SVO(vector<Mesh*> meshes);

private:
    vector<vector<uint>> data;
    int depth;
};