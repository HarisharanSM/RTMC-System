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
    const double a3 = axles.A3 * DEG_TO_RAD;
    const double a4 = axles.A4 * DEG_TO_RAD;
    const Vec3 elbow{BASE_X_M + LINK1_M * std::cos(a1), LINK1_M * std::sin(a1), 0.68};
    const Vec3 eof{elbow.x + LINK2_M * std::cos(a12),
                   elbow.y + LINK2_M * std::sin(a12), 1.20};
    const Mat3 heading = RotationZ(a12);

    FrameTransforms frames{};
    frames[FrameIndex(eBodyFrame::World)] = Pose({0, 0, 0});
    frames[FrameIndex(eBodyFrame::Link1)] = Pose({BASE_X_M, 0, 0.43}, RotationZ(a1));
    frames[FrameIndex(eBodyFrame::Link2)] = Pose(elbow, heading);
    frames[FrameIndex(eBodyFrame::EofSupport)] = Pose({eof.x, eof.y, 0.68}, heading);
    frames[FrameIndex(eBodyFrame::CArm)] = Pose(eof,
        Multiply(Multiply(heading, RotationY(a3)), RotationX(a4)));
    return frames;
}

OrientedBox cBodyKinematics::CalculateBox(const CollisionBody& body,
                                          const AxelPostion& axles) const {
    return CalculateBox(body, CalculateFrames(axles));
}

OrientedBox cBodyKinematics::CalculateBox(const CollisionBody& body,
                                          const FrameTransforms& frames) const {
    const Transform3& frame = frames[FrameIndex(body.frame)];
    const Mat3 localRotation = RotationX(body.rotationXRad);
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
    const double dq3 = AbsAngleDelta(from.A3, to.A3);
    const double dq4 = AbsAngleDelta(from.A4, to.A4);
    const double localRadius = Norm(body.center);

    switch (body.frame) {
        case eBodyFrame::Link1:
            return PlanarRadius(body.center) * dq1;
        case eBodyFrame::Link2:
            return LINK1_M * dq1 + PlanarRadius(body.center) * dq12;
        case eBodyFrame::EofSupport:
            return LINK1_M * dq1 + LINK2_M * dq12 + PlanarRadius(body.center) * dq12;
        case eBodyFrame::CArm:
            return LINK1_M * dq1 + LINK2_M * dq12 +
                   localRadius * (dq12 + dq3 + dq4);
        case eBodyFrame::World:
            break;
    }
    return 0.0;
}

} // namespace RTMCCollision
