#include "svdag.h"
#include "bit_operations.h"
#include "image.h"
#include <iostream>
#include <chrono>
#include <stack>

SVDAG svdag;
size_t max_depth =9;

void outputImage(const SVDAG& svdag);

int main(int, char* argv[]) {

	std::string offFilename = "../../assets/example_highres.off";
	std::string outputFilename = "../../output/output.ppm";
	std::vector<Mesh*> meshes;
	meshes.push_back(new Mesh());
	meshes[0]->loadOFF(offFilename);

	std::cout << "Contains: " << meshes[0]->n_faces() << std::endl;
	AABB bbox = AABB::get_bbox(meshes);
	std::cout << bbox.min_corner << std::endl << bbox.max_corner << std::endl;

	auto t1 = std::chrono::high_resolution_clock::now();
	
	svdag = SVDAG(meshes, Vec3f(-.6f, -.6f, -.6f), Vec3f(.6f, .6f, .6f), max_depth);
	
	auto t2 = std::chrono::high_resolution_clock::now();
	std::cout << "process took:"
		<< std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
		<< " milliseconds\n" << std::endl;
	
	cout<<svdag.shadowRay(Ray(Vec3f(0.215625, 0.234375, 0.046875), normalize(Vec3f(1, 1, 0))), 100)<<endl;
	cout << "success!" << endl;
	outputImage(svdag);
	cout << "Image written" << endl;
	getchar();
	return 0;
};

using ipair = pair<int, int>;
using node = pair<ipair, Vec3i>; // depth,idx + offset in 3D


void outputImage(const SVDAG& svdag) {

	Vec3f black = Vec3f(0, 0, 0);
	Vec3f white = Vec3f(1, 1, 1);
	int resolution = (int)pow(2, max_depth);
	Image image = Image(resolution, resolution, black);
	stack<node> stack;
	stack.push(node(ipair(0, 0), Vec3i()));
	
	while (!stack.empty()) {
		const node n = stack.top();
		stack.pop();
		const int depth = n.first.first;
		const int idx = n.first.second;
		Vec3i offset = n.second;


		if (depth == max_depth - 2) {// Node is a leaf
			const uint64_t leaf = svdag.leaves[idx];
			for (int i : {0, 1, 2, 3}) { // Reverse enumeration compared to encoding
				for (int j : {0, 1, 2, 3}) {
					float value = image.getPixelColor(i + offset[0], j + offset[1])[0];
					for (int k : {0, 1, 2, 3}) {
						if (svdag.readLeaf(leaf, i, j, k))
							value = max(value, (float)(offset[2] + k) / resolution);
					}
					image.setPixelColor(i + offset[0], j + offset[1], value * white);
				}
			}

		}
		else {
		uint childmask = svdag.nodes[depth][idx];
			int ptr_offset = 1;
			for (bool i : {0, 1}) {
				for (bool j : {0, 1}) {
					for (bool k : {0, 1}) {
						if (get_bit(childmask, 4 * i + j * 2 + k)) {
							stack.push(
								node(
									ipair(depth + 1, svdag.nodes[depth][idx+ptr_offset]),
									offset + (int)pow(2, max_depth - 1 - depth) * Vec3i(i,j,k)));
							ptr_offset++;
						}
					}
				}
			}
		}
	}

	Vec3f stride = svdag.bbox.max_corner - svdag.bbox.min_corner;
	stride /= resolution;
	for (int i = 0; i < resolution;i++) {
		for (int j = 0; j < resolution; j++) {
			float value = image.getPixelColor(i, j)[0];
			if (value > 0) {
				Vec3f pos = svdag.bbox.min_corner + stride * Vec3f(i + 0.5f, j + 0.5f, value * resolution + 0.5f);
				if (svdag.shadowRay(Ray(pos, normalize(Vec3f(2, 2, 4) - pos)), 100)) {
					image.setPixelColor(i, j, Vec3f(1, 0, 0));
					/*if (i > 32)
						cout << pos << " "<<i<<" "<< j<<" "<<endl;*/
				}
			}

		}
	}
	image.savePPM("../../output/output.ppm");
}