#include "svo.h"

void compute_SVO(vector<Mesh*> meshes) {
	// Find the min/max corners
	AABB bbox = AABB::get_bbox(meshes);

}