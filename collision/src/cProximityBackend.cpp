#include "cProximityBackend.h"
#include "cCollisionMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace RTMCCollision {
namespace {

std::array<Vec3, 8> Vertices(const OrientedBox& box) {
    std::array<Vec3, 8> result{};
    for (int index = 0; index < 8; ++index) {
        const Vec3 local{
            (index & 4) ? box.halfSize.x : -box.halfSize.x,
            (index & 2) ? box.halfSize.y : -box.halfSize.y,
            (index & 1) ? box.halfSize.z : -box.halfSize.z
        };
        result[static_cast<std::size_t>(index)] = box.center + Multiply(box.orientation, local);
    }
    return result;
}

double PointBoxDistanceSquared(const Vec3& point, const OrientedBox& box) {
    const Vec3 delta = point - box.center;
    double squared = 0.0;
    const double half[] = {box.halfSize.x, box.halfSize.y, box.halfSize.z};
    for (int axis = 0; axis < 3; ++axis) {
        const double coordinate = Dot(delta, Axis(box.orientation, axis));
        const double excess = std::max(0.0, std::abs(coordinate) - half[axis]);
        squared += excess * excess;
    }
    return squared;
}

double SegmentDistanceSquared(const Vec3& p0, const Vec3& p1,
                              const Vec3& q0, const Vec3& q1) {
    constexpr double epsilon = 1e-15;
    const Vec3 d1 = p1 - p0;
    const Vec3 d2 = q1 - q0;
    const Vec3 r = p0 - q0;
    const double a = Dot(d1, d1);
    const double e = Dot(d2, d2);
    const double f = Dot(d2, r);
    double s = 0.0, t = 0.0;
    if (a <= epsilon && e <= epsilon) return Dot(r, r);
    if (a <= epsilon) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = Dot(d1, r);
        if (e <= epsilon) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = Dot(d1, d2);
            const double denominator = a * e - b * b;
            if (denominator > epsilon) s = std::clamp((b * f - c * e) / denominator, 0.0, 1.0);
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    const Vec3 closest = r + d1 * s - d2 * t;
    return Dot(closest, closest);
}

double ExactSurfaceDistance(const OrientedBox& lhs, const OrientedBox& rhs) {
    const auto lhsVertices = Vertices(lhs);
    const auto rhsVertices = Vertices(rhs);
    double best = std::numeric_limits<double>::infinity();
    for (const Vec3& point : lhsVertices) best = std::min(best, PointBoxDistanceSquared(point, rhs));
    for (const Vec3& point : rhsVertices) best = std::min(best, PointBoxDistanceSquared(point, lhs));
    constexpr int edgeAxes[] = {1, 2, 4};
    for (int lhsIndex = 0; lhsIndex < 8; ++lhsIndex) {
        for (int lhsBit : edgeAxes) {
            const int lhsEnd = lhsIndex ^ lhsBit;
            if (lhsIndex > lhsEnd) continue;
            for (int rhsIndex = 0; rhsIndex < 8; ++rhsIndex) {
                for (int rhsBit : edgeAxes) {
                    const int rhsEnd = rhsIndex ^ rhsBit;
                    if (rhsIndex > rhsEnd) continue;
                    best = std::min(best, SegmentDistanceSquared(
                        lhsVertices[static_cast<std::size_t>(lhsIndex)],
                        lhsVertices[static_cast<std::size_t>(lhsEnd)],
                        rhsVertices[static_cast<std::size_t>(rhsIndex)],
                        rhsVertices[static_cast<std::size_t>(rhsEnd)]));
                }
            }
        }
    }
    return std::sqrt(std::max(0.0, best));
}

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

double cProximityBackend::SurfaceDistance(const OrientedBox& lhs,
                                          const OrientedBox& rhs) const {
    const SeparationResult separation = Separation(lhs, rhs);
    return separation.overlapping ? 0.0 : ExactSurfaceDistance(lhs, rhs);
}

} // namespace RTMCCollision
