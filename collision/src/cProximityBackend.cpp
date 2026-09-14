#include "cProximityBackend.h"
#include "cCollisionMath.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace RTMCCollision {
namespace {

double ProjectionRadius(const OrientedBox& box, const Vec3& axis) {
    return box.halfSize.x * std::abs(Dot(Axis(box.orientation, 0), axis)) +
           box.halfSize.y * std::abs(Dot(Axis(box.orientation, 1), axis)) +
           box.halfSize.z * std::abs(Dot(Axis(box.orientation, 2), axis));
}

double AxisGap(const OrientedBox& lhs, const OrientedBox& rhs, const Vec3& axis) {
    const double length = Norm(axis);
    if (length <= 1e-10) return -1.0;
    const Vec3 normalized = axis * (1.0 / length);
    return std::abs(Dot(rhs.center - lhs.center, normalized)) -
           ProjectionRadius(lhs, normalized) - ProjectionRadius(rhs, normalized);
}

} // namespace

SeparationResult cProximityBackend::Separation(const OrientedBox& lhs,
                                               const OrientedBox& rhs) const {
    std::array<Vec3, 15> axes{};
    for (int i = 0; i < 3; ++i) {
        axes[static_cast<std::size_t>(i)] = Axis(lhs.orientation, i);
        axes[static_cast<std::size_t>(3 + i)] = Axis(rhs.orientation, i);
    }
    int index = 6;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            axes[static_cast<std::size_t>(index++)] =
                Cross(Axis(lhs.orientation, i), Axis(rhs.orientation, j));
        }
    }

    double largestGap = 0.0;
    bool separated = false;
    for (const Vec3& axis : axes) {
        const double gap = AxisGap(lhs, rhs, axis);
        if (gap > 0.0) {
            separated = true;
            largestGap = std::max(largestGap, gap);
        }
    }
    return {!separated, largestGap};
}

} // namespace RTMCCollision
