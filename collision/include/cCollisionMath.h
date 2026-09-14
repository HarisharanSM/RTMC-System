#pragma once

#include "cCollisionTypes.h"

namespace RTMCCollision {

double Dot(const Vec3& lhs, const Vec3& rhs);
Vec3 Cross(const Vec3& lhs, const Vec3& rhs);
double Norm(const Vec3& value);
Vec3 Normalize(const Vec3& value);
Mat3 Multiply(const Mat3& lhs, const Mat3& rhs);
Vec3 Multiply(const Mat3& matrix, const Vec3& value);
Transform3 Multiply(const Transform3& lhs, const Transform3& rhs);
Mat3 RotationX(double angleRad);
Mat3 RotationY(double angleRad);
Mat3 RotationZ(double angleRad);
Vec3 Axis(const Mat3& matrix, int column);

} // namespace RTMCCollision
