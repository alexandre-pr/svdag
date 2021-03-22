#include "transform.h"

Transform::Transform() : scale(1) {};

Transform::Transform(const Vec3f& translation, const Vec3f& rotation_axis, float rotation_angle, float scale): translation(translation), scale(scale), rotation(rotation_axis, rotation_angle)
{};

Vec3f Transform::apply(const Vec3f& position) const {
	return rotation.apply((scale * position)) + translation;
};

void Transform::set_rotation(const Vec3f& rotation_axis, float rotation_angle) {
	rotation = Rotation(rotation_axis, rotation_angle);
};

void Transform::set_translation(const Vec3f& t) {
	translation = t;
};

void Transform::set_scale(float s) {
	scale = s;
};

const Rotation& Transform::get_rotation() const {
	return rotation;
}