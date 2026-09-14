#pragma once

#include "cCollisionTypes.h"

#include <array>

namespace RTMCCollision {

class cBodyKinematics {
public:
    using FrameTransforms = std::array<Transform3, 5>;

    FrameTransforms CalculateFrames(const AxelPostion& axles) const;
    OrientedBox CalculateBox(const CollisionBody& body, const AxelPostion& axles) const;
    OrientedBox CalculateBox(const CollisionBody& body, const FrameTransforms& frames) const;
    double BoundBodyMotion(const CollisionBody& body, const AxelPostion& from,
                           const AxelPostion& to) const;

private:
    static std::size_t FrameIndex(eBodyFrame frame);
};

} // namespace RTMCCollision
