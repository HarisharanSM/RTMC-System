#pragma once

#include "cCollisionTypes.h"

namespace RTMCCollision {

struct SeparationResult {
    bool overlapping = false;
    double largestSeparatingGapM = 0.0;
};

class cProximityBackend {
public:
    SeparationResult Separation(const OrientedBox& lhs, const OrientedBox& rhs) const;
};

} // namespace RTMCCollision
