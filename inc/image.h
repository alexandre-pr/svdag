#include <iostream>
#include <fstream>
#include <vector>
#include "mat3.h"

#pragma once

class Image
{
public:
	int width;
	int height;
	std::vector<Vec3f> color;

	Image(int width, int height);
	
	Image(int width, int height, const Vec3f& fill_color);


	void savePPM(std::string filename) const;

	void fillBackground(const Vec3f& color1, const Vec3f& color2);

	void setPixelColor(int i, int j, const Vec3f& pixel_color);

	const Vec3f& getPixelColor(int i, int j) const;

};