#include "rotation.h"
#pragma once

class Transform
{
private:
	Vec3f translation;
	Rotation rotation;
	float scale;
public:
	Transform();

	Transform(const Vec3f& translation, const Vec3f& rotation_axis, float rotation_angle, float scale);

	Vec3f apply(const Vec3f& position) const;

	void set_rotation(const Vec3f& rotation_axis, float rotation_angle);

	void set_translation(const Vec3f& t);

	void set_scale(float s);

	const Rotation& get_rotation() const;

};