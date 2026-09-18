#include "cSceneRegistry.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace RTMCCollision {
namespace {

CollisionBody Box(const std::string& id, const std::string& rigidBody, eBodyFrame frame,
                  const Vec3& center, const Vec3& size, double rotationXRad = 0.0,
                  double rotationYRad = 0.0, double rotationZRad = 0.0,
                  bool obstacle = false) {
    CollisionBody body{id, rigidBody, frame, center, size, rotationXRad, obstacle};
    body.rotationYRad = rotationYRad;
    body.rotationZRad = rotationZRad;
    return body;
}

} // namespace

cSceneRegistry cSceneRegistry::CreateEmptyScene() {
    return {};
}

cSceneRegistry cSceneRegistry::CreateReferenceScene(bool includePatientFixture) {
    cSceneRegistry scene;
    scene.m_Bodies.reserve(48);
#include "../../data/collision/reference/generated/scene_data.inc"
    if (!includePatientFixture) {
        scene.m_Bodies.erase(
            std::remove_if(scene.m_Bodies.begin(), scene.m_Bodies.end(), [](const CollisionBody& body) {
                return body.rigidBody == "patient_fixture";
            }), scene.m_Bodies.end());
        ++scene.m_Generation;
    }
    return scene;
}

void cSceneRegistry::AddBody(const CollisionBody& body) {
    m_Bodies.push_back(body);
    ++m_Generation;
}

void cSceneRegistry::AddStaticObstacle(const std::string& id, const Vec3& center,
                                       const Vec3& size, double rotationXRad) {
    AddBody(Box(id, id, eBodyFrame::World, center, size, rotationXRad, 0.0, 0.0, true));
}

void cSceneRegistry::AddPairExclusion(const std::string& firstRigidBody,
                                      const std::string& secondRigidBody) {
    m_PairExclusions.emplace_back(firstRigidBody, secondRigidBody);
    ++m_Generation;
}

bool cSceneRegistry::IsPairExcluded(const CollisionBody& lhs, const CollisionBody& rhs) const {
    for (const auto& pair : m_PairExclusions) {
        if ((lhs.rigidBody == pair.first && rhs.rigidBody == pair.second) ||
            (lhs.rigidBody == pair.second && rhs.rigidBody == pair.first)) return true;
    }
    return false;
}

bool cSceneRegistry::IsValid() const {
    std::set<std::string> ids;
    std::set<std::string> rigidBodies;
    for (const auto& body : m_Bodies) {
        if (body.id.empty() || body.rigidBody.empty() || !ids.insert(body.id).second) return false;
        if (!std::isfinite(body.center.x) || !std::isfinite(body.center.y) ||
            !std::isfinite(body.center.z) || !std::isfinite(body.size.x) ||
            !std::isfinite(body.size.y) || !std::isfinite(body.size.z) ||
            !std::isfinite(body.rotationXRad) || !std::isfinite(body.rotationYRad) ||
            !std::isfinite(body.rotationZRad)) return false;
        if (body.size.x <= 0.0 || body.size.y <= 0.0 || body.size.z <= 0.0) return false;
        if (body.obstacle && body.frame != eBodyFrame::World) return false;
        rigidBodies.insert(body.rigidBody);
    }
    std::set<std::pair<std::string, std::string>> exclusions;
    for (auto pair : m_PairExclusions) {
        if (pair.first.empty() || pair.second.empty() || pair.first == pair.second ||
            !rigidBodies.count(pair.first) || !rigidBodies.count(pair.second)) return false;
        if (pair.second < pair.first) std::swap(pair.first, pair.second);
        if (!exclusions.insert(pair).second) return false;
    }
    return true;
}

} // namespace RTMCCollision
