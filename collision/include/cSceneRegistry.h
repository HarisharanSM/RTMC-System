#pragma once

#include "cCollisionTypes.h"

#include <cstdint>
#include <vector>

namespace RTMCCollision {

class cSceneRegistry {
public:
    cSceneRegistry() = default;

    static cSceneRegistry CreateReferenceScene(bool includePatientFixture = true);
    static cSceneRegistry CreateEmptyScene();

    void AddBody(const CollisionBody& body);
    void AddStaticObstacle(const std::string& id, const Vec3& center, const Vec3& size,
                           double rotationXRad = 0.0);

    const std::vector<CollisionBody>& Bodies() const { return m_Bodies; }
    std::uint64_t Generation() const { return m_Generation; }
    bool IsValid() const;

private:
    std::vector<CollisionBody> m_Bodies;
    std::uint64_t m_Generation = 1;
};

} // namespace RTMCCollision
