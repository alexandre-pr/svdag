#include "scene/mesh.h"
#include <vector>

Mesh::Mesh() {};

Mesh::Mesh(std::vector<Vec3f> vertices, std::vector<Vec3i> faces) : vertices(vertices), faces(faces) {
};

void Mesh::loadOFF(std::string filename) {
	std::ifstream ifs(filename);
	std::string line;
	int count = 0;
	int initial_n_vert = n_vertices();
	int n_vert;
	int n_face;
	std::istringstream ss;

	while (std::getline(ifs, line))  //read each line of the file
	{
		if (line[0] == '%') //line is a comment
		{
			continue;
		}
		else {
			count++;
		}

		// Header
		if (count == 2) {
			ss = std::istringstream(line);
			ss >> n_vert >> n_face;
			break;
		}
	}

	// Reading data
	count = 0;
	while (std::getline(ifs, line))  //read each line of the file
	{
		if (line[0] == '%') //line is a comment
		{
			continue;
		}
		else {
			count++;
		}

		if (count <= n_vert) {
			ss = std::istringstream(line);
			float x, y, z;
			ss >> x >> y >> z;
			add_vertice(Vec3f(x, y, z));
		}
		else {
			ss = std::istringstream(line);
			int x;
			ss >> x;
			assert(x == 3);
			int i, j, k;
			ss >> i >> j >> k;
			add_face(Vec3i(i, j, k) + Vec3i(initial_n_vert, initial_n_vert, initial_n_vert));
		}
	}

	compute_normals();
	//compute_bbox();
}


void Mesh::compute_normals() {
	normals = std::vector<Vec3f>(n_vertices(), Vec3f(0,0,0));
	
	for (int i = 0; i < n_faces(); i++) {
		Vec3i f = get_face(i);
		Vec3f p0 = get_vertice(f[0]);
		Vec3f p1 = get_vertice(f[1]);
		Vec3f p2 = get_vertice(f[2]);
		Vec3f p01 = p1 - p0;
		Vec3f p02 = p2 - p0;
		Vec3f p12 = p2 - p1;
		Vec3f n = cross(p01, p02);
		float area = length(n); // *0.5
		n = n / area;

		// Area based weighting
		for (int k = 0; k < 3; k++) {
			normals[f[k]] += n * area;
		}
	}

	for (int i = 0; i < n_vertices(); i++) {
		//if (normals[i] == Vec3f(0, 0, 0)) { std::cout << "PB"; }
		normals[i] = normalize(normals[i]);
	}
}

void Mesh::apply_transform(const Transform& transform) {
	for (size_t i = 0; i < n_vertices(); i++) {
		vertices[i] = transform.apply(vertices[i]);
	}
	Rotation rotation = transform.get_rotation();
	for (size_t i = 0; i < n_vertices(); i++) {
		normals[i] = rotation.apply(normals[i]);
	}
}

Mesh square_primitive(const Vec3f& p00, const Vec3f& p10,
	const Vec3f& p01, const Vec3f& p11) {
	Mesh square;
	square.add_vertice(p00);
	square.add_vertice(p10);
	square.add_vertice(p01);
	square.add_vertice(p11);
	square.add_face(Vec3i(0, 3, 2));
	square.add_face(Vec3i(0, 1, 3));
	square.compute_normals();
	//square.compute_bbox();
	return square;
}