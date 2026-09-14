#include "cSceneRegistry.h"

#include <cmath>
#include <set>

namespace RTMCCollision {
namespace {

constexpr double PI = 3.14159265358979323846;

CollisionBody Box(const std::string& id, const std::string& rigidBody, eBodyFrame frame,
                  const Vec3& center, const Vec3& size, double rotationXRad = 0.0,
                  bool obstacle = false) {
    return {id, rigidBody, frame, center, size, rotationXRad, obstacle};
}

} // namespace

cSceneRegistry cSceneRegistry::CreateEmptyScene() {
    return {};
}

cSceneRegistry cSceneRegistry::CreateReferenceScene(bool includePatientFixture) {
    cSceneRegistry scene;
    scene.m_Bodies.reserve(40);

    scene.AddBody(Box("link1_housing", "link1", eBodyFrame::Link1,
                      {0.375, 0, 0}, {0.75, 0.20, 0.16}));
    scene.AddBody(Box("elbow_housing", "link1", eBodyFrame::Link1,
                      {0.75, 0, 0.125}, {0.24, 0.24, 0.25}));
    scene.AddBody(Box("link2_housing", "link2", eBodyFrame::Link2,
                      {0.50, 0, 0}, {1.00, 0.16, 0.14}));
    scene.AddBody(Box("support_rear_beam", "link2", eBodyFrame::EofSupport,
                      {0, -0.45, 0}, {0.16, 1.06, 0.16}));
    scene.AddBody(Box("support_column", "link2", eBodyFrame::EofSupport,
                      {0, -0.90, 0.26}, {0.16, 0.16, 0.52}));

    constexpr int segments = 24;
    constexpr double inner = 0.70;
    constexpr double outer = 0.84;
    const double start = PI / 2.0;
    const double end = 3.0 * PI / 2.0;
    const double half = (end - start) / (2.0 * segments);
    const double radialMin = inner * std::cos(half);
    const double radialMax = outer;
    const double radialCenter = (radialMin + radialMax) / 2.0;
    for (int i = 0; i < segments; ++i) {
        const double theta = start + (2.0 * i + 1.0) * half;
        scene.AddBody(Box("carm_sector_" + std::to_string(i), "carm", eBodyFrame::CArm,
                          {0, radialCenter * std::cos(theta), radialCenter * std::sin(theta)},
                          {0.22, radialMax - radialMin, 2.0 * outer * std::sin(half)}, theta));
    }
    scene.AddBody(Box("detector_housing", "carm", eBodyFrame::CArm,
                      {0, 0, 0.60}, {0.48, 0.42, 0.245}));
    scene.AddBody(Box("source_housing", "carm", eBodyFrame::CArm,
                      {0, 0, -0.62}, {0.38, 0.40, 0.285}));

    scene.AddBody(Box("table_top", "fixed_table", eBodyFrame::World,
                      {0.90, 0, 0.90}, {2.20, 0.55, 0.10}, 0, true));
    scene.AddBody(Box("table_pedestal", "fixed_table", eBodyFrame::World,
                      {1.70, 0, 0.475}, {0.40, 0.45, 0.75}, 0, true));
    scene.AddBody(Box("table_base", "fixed_table", eBodyFrame::World,
                      {1.70, 0, 0.05}, {0.90, 0.80, 0.10}, 0, true));
    scene.AddBody(Box("mattress", "fixed_table", eBodyFrame::World,
                      {0.90, 0, 0.975}, {2.00, 0.53, 0.05}, 0, true));
    if (includePatientFixture) {
        scene.AddBody(Box("patient_test_envelope", "patient_fixture", eBodyFrame::World,
                          {0.90, 0, 1.15}, {1.80, 0.50, 0.30}, 0, true));
    }
    scene.AddBody(Box("floor", "floor", eBodyFrame::World,
                      {0.50, 0, -0.05}, {6.00, 5.00, 0.10}, 0, true));
    return scene;
}

void cSceneRegistry::AddBody(const CollisionBody& body) {
    m_Bodies.push_back(body);
    ++m_Generation;
}

void cSceneRegistry::AddStaticObstacle(const std::string& id, const Vec3& center,
                                       const Vec3& size, double rotationXRad) {
    AddBody(Box(id, id, eBodyFrame::World, center, size, rotationXRad, true));
}

bool cSceneRegistry::IsValid() const {
    std::set<std::string> ids;
    for (const auto& body : m_Bodies) {
        if (body.id.empty() || body.rigidBody.empty() || !ids.insert(body.id).second) return false;
        if (!std::isfinite(body.center.x) || !std::isfinite(body.center.y) ||
            !std::isfinite(body.center.z) || !std::isfinite(body.size.x) ||
            !std::isfinite(body.size.y) || !std::isfinite(body.size.z) ||
            !std::isfinite(body.rotationXRad)) return false;
        if (body.size.x <= 0.0 || body.size.y <= 0.0 || body.size.z <= 0.0) return false;
        if (body.obstacle && body.frame != eBodyFrame::World) return false;
    }
    return true;
}

} // namespace RTMCCollision
