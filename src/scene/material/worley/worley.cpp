#include "scene/material/worley/worley.h"

Worley::Worley(int N_samples) : N_samples(N_samples)
{
	for (int i = 0; i < N_samples; i++) {
		float x = std::rand() / ((float)RAND_MAX);
		float y = std::rand() / ((float)RAND_MAX);
		float z = std::rand() / ((float)RAND_MAX);
		samples.push_back(Vec3f(x, y, z));
	}
};

float Worley::evaluate(const Vec3f& pos) const {
	Vec3f p = pos;
	for (int i = 0; i < 3; i++) {
		int j = (int)p[i];
		if (p[i] >= 0) {
			p[i] = p[i] - j;
			if (j % 2 == 1) {
				p[i] = 1 - p[i];
			}
		}
		else {
			p[i] = j - p[i];
			if (j % 2 == -1) {
				p[i] = 1 - p[i];
			}
		}
	}
	float d_1 = std::numeric_limits<float>::max();
	float d_2 = std::numeric_limits<float>::max();
	for (Vec3f s : samples) {
		float d = length(s - p);
		if (d < d_1) {
			d_2 = d_1;
			d_1 = d;
		}
		else {
			if (d < d_2)
				d_2 = d;
		}
	}
	return d_2;
}