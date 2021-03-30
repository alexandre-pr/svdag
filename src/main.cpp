#include <iostream>
#include <chrono>
#include <stack>

#include "acceleration_structure/svdag.h"
#include "input_output/image.h"
#include "scene/scene.h"
#include "ray_tracer/ray_tracer.h"

Vec3f blue = Vec3f(0, 0, 1);
Vec3f white = Vec3f(0, 0, 0);
string offFilename = "../../assets/example_highres.off";
string outputFilename = "../../output/output_raytracer.ppm";

// SVDAG
SVDAG svdag;
size_t max_depth =11;

// Ray tracing
int resolution = 400; // width
Scene scene;
Mesh mesh;
Mesh ground;
BRDF m_mesh;
BRDF m_ground;
SquareLightSource light;


void outputSVDAGImage(const SVDAG& svdag);
void computeSVDAG(const Scene& scene, SVDAG& svdag);
void initializeScene(Scene& scene);

int main(int, char* argv[]) {
	//Initializing scene
	initializeScene(scene);

	Image image(resolution, (int)(resolution / scene.camera.get_aspect_ratio()));
	image.fillBackground(white, blue);

	computeSVDAG(scene, svdag);
	outputSVDAGImage(svdag);

	RayTracer shader(16, 3, 16, 16);
	
	
	auto t1 = chrono::high_resolution_clock::now();
	cout << "Raytracing...";
	shader.rayTrace(scene, image, svdag);
	cout << "done" << endl;
	auto t2 = chrono::high_resolution_clock::now();
	cout << "process took: "
		<< chrono::duration_cast<chrono::milliseconds>(t2 - t1).count()
		<< " milliseconds\n";
	
	image.savePPM(outputFilename);
	getchar();
	return 0;
};

void initializeScene(Scene& scene) {
	scene = Scene(Vec3f(.4f, .4f, .4f)); // Ambiant color

	// Camera
	scene.camera.set_position(Vec3f(0, 0.2f, 1.5));
	scene.camera.set_direction(normalize(Vec3f(0, -0.2f, -1)));
	scene.camera.set_direction(normalize(Vec3f(0, 0, -1)));
	scene.camera.set_aspect_ratio(0.7f);
	scene.camera.set_near(0.5f);
	
	// Mesh
	cout << "Loading mesh" << offFilename << "...";
	mesh.loadOFF(offFilename);
	cout << "Done" << endl;
	scene.add_mesh(&mesh);
	ground = square_primitive();
	ground.set_rotation(Vec3f(0, 0, 1), -3.14f / 4);
	ground.set_scale(1.5);
	ground.set_translation(Vec3f(-.5f, -.5f, 0));
	scene.add_mesh(&ground);

	// Material
	m_ground = BRDF(Vec3f(0.1f, 0.5f, .8f), 0.5f, 1, 1);
	scene.add_material(&m_mesh);
	scene.add_material(&m_ground);
	scene.set_material(0, 0); // Setting material for mesh
	scene.set_material(1, 1);

	// Light
	light = SquareLightSource(Vec3f(2.0, 2.0, 1),
		normalize(Vec3f(-1., -1., -1.)), 2.0f, Vec3f(1, 1, 1), 2.0f);
	scene.add_light(&light);

	scene.compute_BVH();
}




void computeSVDAG(const Scene& scene, SVDAG& svdag) {
	auto t1 = chrono::high_resolution_clock::now();

	svdag = SVDAG(scene.meshes, max_depth);

	auto t2 = chrono::high_resolution_clock::now();
	cout << "Process took:"
		<< chrono::duration_cast<chrono::milliseconds>(t2 - t1).count()
		<< " milliseconds\n" << endl;

	cout << "Bounding box: "<<svdag.bbox.min_corner << endl << svdag.bbox.max_corner << endl;

}




using ipair = pair<int, int>;
using node = pair<ipair, Vec3i>; // depth,idx + offset in 3D

void outputSVDAGImage(const SVDAG& svdag) {

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
					float value = image.getPixelColor(i + offset[0], resolution-(j + offset[1]))[0];
					for (int k : {0, 1, 2, 3}) {
						if (svdag.readLeaf(leaf, i, j, k))
							value = max(value, (float)(offset[2] + k) / resolution);
					}
					image.setPixelColor(i + offset[0], resolution - (j + offset[1]), value * white);
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

	// To display the pixels in shadow in red
	Vec3f stride = svdag.bbox.max_corner - svdag.bbox.min_corner;
	stride /= (float)resolution;
	for (int i = 0; i < resolution;i++) {
		for (int j = 0; j < resolution; j++) {
			float value = image.getPixelColor(i, resolution - j)[0];
			if (value > 0) {
				Vec3f pos = svdag.bbox.min_corner + stride * Vec3f((float)i, (float)j, value * resolution);
				Vec3f wi = normalize(Vec3f(2, 2, 4) - pos);
				if (svdag.shadowRay(Ray(pos + 16 * dot(svdag.min_stride, wi) * wi, wi), 100)) {
						image.setPixelColor(i, resolution - j, Vec3f(1, 0, 0));
					/*if (i > 32)
						cout << pos << " "<<i<<" "<< j<<" "<<endl;*/
				}
			}

		}
	}
	image.savePPM("../../output/output_svdag.ppm");
	cout << "Image written" << endl;
}