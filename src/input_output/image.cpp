#include "input_output/image.h"

Image::Image(int width, int height) :
	width(width),
	height(height),
	color(width* height, Vec3f(0, 0, 0))
{};

Image::Image(int width, int height, const Vec3f& fill_color) :
	width(width),
	height(height),
	color(width* height)
{
	for (int i = 0; i < width * height; i++) {
		color[i] = fill_color;
	}
};

void Image::savePPM(std::string filename) const {
	std::ofstream myfile;
	myfile.open(filename);
	myfile << "P3\n";
	myfile << width << " " << height << "\n255\n";

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			Vec3f c = color[(i * height + j)];
			myfile << (int)(255 * c[0]) << " "
				<< (int)(255 * c[1]) << " "
				<< (int)(255 * c[2]) << " ";
		}
		myfile << "\n";
	}
	myfile.close();
}

void Image::fillBackground(const Vec3f& color1, const Vec3f& color2) {
	//#pragma omp parallel for
	for (int ij = 0; ij < width * height; ij++) {
		int i = ij / height;
		int j = ij % height;
		color[height * i + j] = Vec3f((color1[0] * i) / width + (color2[0] * (width - i)) / width,
			(color1[1] * i) / width + (color2[1] * (width - i)) / width,
			(color1[2] * i) / width + (color2[2] * (width - i)) / width);
	}
}

void Image::setPixelColor(int i, int j, const Vec3f& pixel_color) {
	color[height * i + j] = clamp(pixel_color,0.f,1.f);
}

const Vec3f& Image::getPixelColor(int i, int j) const {
	return color[height * i + j];
}