#include "rotation.h"
#pragma once

class Transform
{
private:
	Vec3f translation;
	Rotation rotation;
	float scale;
public:
	inline Transform() : scale(1) {};

	inline Transform::Transform(const Vec3f& translation, const Vec3f& rotation_axis, float rotation_angle, float scale) : translation(translation), scale(scale), rotation(rotation_axis, rotation_angle) {};
	
	inline Vec3f Transform::apply(const Vec3f& position) const {
		return rotation.apply((scale * position)) + translation;
	};

	inline void Transform::set_rotation(const Vec3f& rotation_axis, float rotation_angle) {
		rotation = Rotation(rotation_axis, rotation_angle);
	};

	inline void Transform::set_translation(const Vec3f& t) {
		translation = t;
	};

	inline void Transform::set_scale(float s) {
		scale = s;
	};

	inline const Rotation& Transform::get_rotation() const {
		return rotation;
	}

};