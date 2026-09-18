#include "cCollisionMath.h"

#include <cmath>

namespace RTMCCollision {

double Dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 Cross(const Vec3& lhs, const Vec3& rhs) {
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

double Norm(const Vec3& value) { return std::sqrt(Dot(value, value)); }

Vec3 Normalize(const Vec3& value) {
    const double norm = Norm(value);
    return norm > 1e-12 ? value * (1.0 / norm) : Vec3{};
}

Mat3 Multiply(const Mat3& lhs, const Mat3& rhs) {
    Mat3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.m[row][column] = 0.0;
            for (int k = 0; k < 3; ++k) {
                result.m[row][column] += lhs.m[row][k] * rhs.m[k][column];
            }
        }
    }
    return result;
}

Vec3 Multiply(const Mat3& matrix, const Vec3& value) {
    return {
        matrix.m[0][0] * value.x + matrix.m[0][1] * value.y + matrix.m[0][2] * value.z,
        matrix.m[1][0] * value.x + matrix.m[1][1] * value.y + matrix.m[1][2] * value.z,
        matrix.m[2][0] * value.x + matrix.m[2][1] * value.y + matrix.m[2][2] * value.z
    };
}

Transform3 Multiply(const Transform3& lhs, const Transform3& rhs) {
    return {Multiply(lhs.rotation, rhs.rotation),
            lhs.translation + Multiply(lhs.rotation, rhs.translation)};
}

Mat3 RotationX(double angleRad) {
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    Mat3 result{};
    result.m = {{{{1, 0, 0}}, {{0, c, -s}}, {{0, s, c}}}};
    return result;
}

Mat3 RotationY(double angleRad) {
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    Mat3 result{};
    result.m = {{{{c, 0, s}}, {{0, 1, 0}}, {{-s, 0, c}}}};
    return result;
}

Mat3 RotationZ(double angleRad) {
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    Mat3 result{};
    result.m = {{{{c, -s, 0}}, {{s, c, 0}}, {{0, 0, 1}}}};
    return result;
}

Vec3 Axis(const Mat3& matrix, int column) {
    return {matrix.m[0][column], matrix.m[1][column], matrix.m[2][column]};
}

} // namespace RTMCCollision
