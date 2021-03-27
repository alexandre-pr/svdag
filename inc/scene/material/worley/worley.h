#pragma once

#include <vector>
#include <iostream>

#include "math/vec3.h"
#include "helper/sampling.h"


// Really minimal (i.e bad) implementation of Worley noise
class Worley {
public:
	Worley(int N_samples = 10);

	float evaluate(const Vec3f& pos) const;

private:
	int N_samples;
	std::vector<Vec3f> samples;
};