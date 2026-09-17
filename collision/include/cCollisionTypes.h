#pragma once

#include "commonDrive.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace RTMCCollision {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
};

struct Mat3 {
    std::array<std::array<double, 3>, 3> m{{{{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}}};
};

struct Transform3 {
    Mat3 rotation{};
    Vec3 translation{};
};

enum class eBodyFrame {
    World,
    Link1,
    Link2,
    Column,
    Boom,
    A4Carrier,
    A5Carrier,
    CArm
};

struct CollisionBody {
    std::string id;
    std::string rigidBody;
    eBodyFrame frame = eBodyFrame::World;
    Vec3 center{};
    Vec3 size{};
    double rotationXRad = 0.0;
    bool obstacle = false;
    double rotationYRad = 0.0;
    double rotationZRad = 0.0;
};

struct OrientedBox {
    Vec3 center{};
    Vec3 halfSize{};
    Mat3 orientation{};
    const CollisionBody* source = nullptr;
};

enum class eCollisionVerdict {
    Clear,
    Hazard,
    Unknown
};

inline const char* ToString(eCollisionVerdict verdict) {
    switch (verdict) {
        case eCollisionVerdict::Clear: return "Clear";
        case eCollisionVerdict::Hazard: return "Hazard";
        case eCollisionVerdict::Unknown: return "Unknown";
    }
    return "Unknown";
}

struct CollisionRequest {
    std::uint64_t session = 0;
    std::uint64_t sequence = 0;
    std::uint64_t sceneGeneration = 0;
    drivePosition currentPosition{};
    AxelPostion currentAxles{};
    joystickSignal direction{};
    double linearSpeedMps = 0.0;
    double angularSpeedRadps = 0.0;
    bool velocityMeasured = false;
};

struct CollisionPermit {
    std::uint64_t session = 0;
    std::uint64_t sequence = 0;
    std::uint64_t sceneGeneration = 0;
    eCollisionVerdict verdict = eCollisionVerdict::Unknown;
    std::chrono::steady_clock::time_point expiresAt{};
    double predictedTravelM = 0.0;
    double predictedTravelRad = 0.0;
    const char* movingBody = "";
    const char* obstacle = "";
    const char* reason = "";
};

struct PredictionSettings {
    double reactionTimeS = 0.250; // Includes 150 ms renewal expiry and dispatch allowance.
    double maximumLinearSpeedMps = 0.20;
    double maximumLinearAccelerationMps2 = 0.40;
    double guaranteedLinearDecelerationMps2 = 0.50;
    double maximumAngularSpeedRadps = 1.0471975511965976;
    double maximumAngularAccelerationRadps2 = 2.0943951023931953;
    double guaranteedAngularDecelerationRadps2 = 2.0943951023931953;
    double maximumA3SpeedRadps = 0.17453292519943295; // 10 deg/s
    double maximumA3AccelerationRadps2 = 0.3490658503988659; // 20 deg/s2
    double guaranteedA3DecelerationRadps2 = 0.3490658503988659;
    // Residual surface clearance after the complete predicted stop. This is
    // one full-speed 50 ms linear command step (1 cm), not the stopping range.
    double pairMarginM = 0.010;
    double permitLifetimeS = 0.150;
    double intervalMotionToleranceM = 0.0005;
    // Folded-home joint motion changes as sqrt(Cartesian travel); extra depth
    // is needed to certify close but separated head-side support geometry.
    int maximumSubdivisionDepth = 20;
};

} // namespace RTMCCollision
