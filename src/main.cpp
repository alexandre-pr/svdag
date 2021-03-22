#include "svo.h"
#include "bit_operations.h"
#include "image.h"
#include <iostream>
#include <chrono>
#include <stack>

SVO svo;
size_t max_depth = 6;

void outputImage();

int main(int, char* argv[]) {

	std::string offFilename = "../../assets/example_lowres.off";
	std::string outputFilename = "../../output/output.ppm";
	std::vector<Mesh*> meshes;
	meshes.push_back(new Mesh());
	meshes[0]->loadOFF(offFilename);

	std::cout << "Contains: " << meshes[0]->n_faces() << std::endl;
	AABB bbox = AABB::get_bbox(meshes);
	std::cout << bbox.min_corner << std::endl << bbox.max_corner << std::endl;

	auto t1 = std::chrono::high_resolution_clock::now();
	
	svo = SVO(meshes, max_depth);
	
	auto t2 = std::chrono::high_resolution_clock::now();
	std::cout << "process took:"
		<< std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
		<< " milliseconds\n";

	size_t size = 0;
	for (size_t i = 0; i < svo.nodes.size(); i++) {
		size += svo.nodes[i].size() * 4;
	}

	outputImage();
	return 0;
};

using ipair = pair<int, int>;
using node = pair<ipair, Vec3i>; // depth,idx + offset in 3D


void outputImage() {
	Vec3f black = Vec3f(0, 0, 0);
	Vec3f white = Vec3f(1, 1, 1);
	size_t resolution = pow(2, max_depth);
	Image image = Image(resolution, resolution, black);
	stack<node> stack;
	stack.push(node(ipair(0, 0), Vec3i()));
	
	while (!stack.empty()) {
		node n = stack.top();
		stack.pop();
		int depth = n.first.first;
		int idx = n.first.second;
		Vec3i offset = n.second;


		uint childmask = svo.nodes[depth][idx];
		if (depth == max_depth - 1) {// Node before leaves
			for (bool i : {0, 1}) {
				for (bool j : {0, 1}) {
					if (get_bit(childmask, i + j * 2)) {
						Vec3f color = image.getPixelColor(i + offset[0], j + offset[1]);
						float value = max(color[0], (float)offset[2] / resolution);
						image.setPixelColor(i+offset[0], j+offset[1], value * white);
					}
					if (get_bit(childmask, i + j * 2 + 4)) {
						Vec3f color = image.getPixelColor(i + offset[0], j + offset[1]);
						float value = max(color[0], (float)(1 + offset[2])/resolution);
						image.setPixelColor(i+offset[0], j+offset[1], value * white);
					}
				}
			}
		}
		else {
			int ptr_offset = 1;
			for (bool i : {0, 1}) {
				for (bool j : {0, 1}) {
					for (bool k : {0, 1}) {
						if (get_bit(childmask, i + j * 2 + k * 4)) {
							stack.push(
								node(
									ipair(depth + 1, svo.nodes[depth][idx+ptr_offset]),
									offset + (int)pow(2, max_depth - 1 - depth) * Vec3i(i,j,k)));
							ptr_offset++;
						}
					}
				}
			}
		}
	}
	image.savePPM("../../output/output.ppm");
}