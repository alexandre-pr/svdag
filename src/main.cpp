#include "svo.h"
#include "bit_operations.h"
#include <iostream>
#include <chrono>


int main(int, char* argv[]) {

	std::string offFilename = "../../assets/example_lowres.off";
	std::string outputFilename = "../../output/output.ppm";
	std::vector<Mesh*> meshes;
	meshes.push_back(new Mesh());
	meshes[0]->loadOFF(offFilename);
	
	std::cout << "Contains: " << meshes[0]->n_vertices() << std::endl;
	AABB bbox = AABB::get_bbox(meshes);
	std::cout << bbox.min_corner << std::endl << bbox.max_corner << std::endl;
	

	auto t1 = std::chrono::high_resolution_clock::now();
	auto t2 = std::chrono::high_resolution_clock::now();
	std::cout << "process took: "
		<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()
		<< " microseconds\n";
	return 0;
};