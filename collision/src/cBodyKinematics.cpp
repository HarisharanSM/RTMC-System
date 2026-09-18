#include "cBodyKinematics.h"
#include "cCollisionMath.h"

#include <algorithm>
#include <cmath>

namespace RTMCCollision {
namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double LINK1_M = 0.75;
constexpr double LINK2_M = 1.00;
constexpr double BASE_X_M = -0.25;
constexpr double WORLD_FROM_PLANAR_X_M = -1.15;
constexpr double LINK1_Z_M = 0.10;
constexpr double LINK2_Z_M = 0.28;
constexpr double COLUMN_FOOT_Z_M = 0.37;
constexpr double ISOCENTER_Z_M = 1.20;
constexpr double COLUMN_TO_ISOCENTER_X_M = 1.15;

Transform3 Pose(const Vec3& translation, const Mat3& rotation = Mat3{}) {
    return {rotation, translation};
}

double PlanarRadius(const Vec3& center) {
    return std::hypot(center.x, center.y);
}

double AbsAngleDelta(double lhsDeg, double rhsDeg) {
    return std::abs(rhsDeg - lhsDeg) * DEG_TO_RAD;
}

} // namespace

std::size_t cBodyKinematics::FrameIndex(eBodyFrame frame) {
    return static_cast<std::size_t>(frame);
}

cBodyKinematics::FrameTransforms cBodyKinematics::CalculateFrames(const AxelPostion& axles) const {
    const double a1 = axles.A1 * DEG_TO_RAD;
    const double a12 = (axles.A1 + axles.A2) * DEG_TO_RAD;
    const double a123 = (axles.A1 + axles.A2 + axles.A3) * DEG_TO_RAD;
    const double a4 = axles.A4 * DEG_TO_RAD;
    const double a5 = axles.A5 * DEG_TO_RAD;
    const Vec3 elbow{WORLD_FROM_PLANAR_X_M + BASE_X_M + LINK1_M * std::cos(a1),
                     LINK1_M * std::sin(a1), LINK2_Z_M};
    const Vec3 linkEnd{elbow.x + LINK2_M * std::cos(a12),
                       elbow.y + LINK2_M * std::sin(a12), LINK2_Z_M};
    const Mat3 heading = RotationZ(a12);
    const Mat3 aligned = RotationZ(a123);
    const Vec3 imagingCenter = Vec3{linkEnd.x, linkEnd.y, ISOCENTER_Z_M} +
        Multiply(aligned, Vec3{COLUMN_TO_ISOCENTER_X_M, 0, 0});

    FrameTransforms frames{};
    frames[FrameIndex(eBodyFrame::World)] = Pose({0, 0, 0});
    frames[FrameIndex(eBodyFrame::Link1)] = Pose(
        {WORLD_FROM_PLANAR_X_M + BASE_X_M, 0, LINK1_Z_M}, RotationZ(a1));
    frames[FrameIndex(eBodyFrame::Link2)] = Pose(elbow, heading);
    frames[FrameIndex(eBodyFrame::Column)] = Pose(
        {linkEnd.x, linkEnd.y, COLUMN_FOOT_Z_M}, heading);
    frames[FrameIndex(eBodyFrame::Boom)] = Pose(
        {linkEnd.x, linkEnd.y, ISOCENTER_Z_M}, aligned);
    const Mat3 a4Rotation = Multiply(aligned, RotationX(a4));
    const Mat3 a5Rotation = Multiply(a4Rotation, RotationY(a5));
    frames[FrameIndex(eBodyFrame::A4Carrier)] = Pose(imagingCenter, a4Rotation);
    frames[FrameIndex(eBodyFrame::A5Carrier)] = Pose(imagingCenter, a5Rotation);
    frames[FrameIndex(eBodyFrame::CArm)] = Pose(imagingCenter, a5Rotation);
    return frames;
}

OrientedBox cBodyKinematics::CalculateBox(const CollisionBody& body,
                                          const AxelPostion& axles) const {
    return CalculateBox(body, CalculateFrames(axles));
}

OrientedBox cBodyKinematics::CalculateBox(const CollisionBody& body,
                                          const FrameTransforms& frames) const {
    const Transform3& frame = frames[FrameIndex(body.frame)];
    const Mat3 localRotation = Multiply(
        Multiply(RotationX(body.rotationXRad), RotationY(body.rotationYRad)),
        RotationZ(body.rotationZRad));
    return {
        frame.translation + Multiply(frame.rotation, body.center),
        body.size * 0.5,
        Multiply(frame.rotation, localRotation),
        &body
    };
}

double cBodyKinematics::BoundBodyMotion(const CollisionBody& body, const AxelPostion& from,
                                        const AxelPostion& to) const {
    if (body.frame == eBodyFrame::World) return 0.0;

    const double dq1 = AbsAngleDelta(from.A1, to.A1);
    const double dq12 = std::abs((to.A1 + to.A2) - (from.A1 + from.A2)) * DEG_TO_RAD;
    const double dq123 = std::abs((to.A1+to.A2+to.A3)-(from.A1+from.A2+from.A3)) * DEG_TO_RAD;
    const double dq4 = AbsAngleDelta(from.A4, to.A4);
    const double dq5 = AbsAngleDelta(from.A5, to.A5);
    // Include every corner, not only the primitive center.
    const double localRadius = Norm(body.center) + Norm(body.size * 0.5);
    const double planarRadius = PlanarRadius(body.center) + Norm(body.size * 0.5);

    switch (body.frame) {
        case eBodyFrame::Link1:
            return planarRadius * dq1;
        case eBodyFrame::Link2:
            return LINK1_M * dq1 + planarRadius * dq12;
        case eBodyFrame::Column:
            return LINK1_M * dq1 + LINK2_M * dq12 + planarRadius * dq12;
        case eBodyFrame::Boom:
            return LINK1_M * dq1 + LINK2_M * dq12 + localRadius * dq123;
        case eBodyFrame::A4Carrier:
            return LINK1_M * dq1 + LINK2_M * dq12 +
                   COLUMN_TO_ISOCENTER_X_M * dq123 + localRadius * (dq123 + dq4);
        case eBodyFrame::A5Carrier:
        case eBodyFrame::CArm:
            return LINK1_M * dq1 + LINK2_M * dq12 +
                   COLUMN_TO_ISOCENTER_X_M * dq123 + localRadius * (dq123 + dq4 + dq5);
        case eBodyFrame::World:
            break;
    }
    return 0.0;
}

} // namespace RTMCCollision
