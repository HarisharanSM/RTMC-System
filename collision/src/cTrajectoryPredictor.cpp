#include "cTrajectoryPredictor.h"

#include "cDriveCalculator.h"

#include <algorithm>
#include <cmath>

namespace RTMCCollision {
namespace {

constexpr double PI = 3.14159265358979323846;

double StoppingTravel(double speed, bool velocityMeasured, double maximumSpeed,
                      double acceleration, double reactionTime, double braking) {
    speed = std::min(std::abs(speed), maximumSpeed);
    // The simulator has commanded pose but no measured velocity. Until a motion
    // backend supplies bounded measured feedback, use maximum speed.
    if (!velocityMeasured) speed = maximumSpeed;
    const double speedAtBrake = speed + acceleration * reactionTime;
    const double reactionTravel = speed * reactionTime + 0.5 * acceleration * reactionTime * reactionTime;
    return reactionTravel + speedAtBrake * speedAtBrake / (2.0 * braking);
}

bool SamePair(const CollisionBody& lhs, const CollisionBody& rhs,
              const char* first, const char* second) {
    return (lhs.rigidBody == first && rhs.rigidBody == second) ||
           (lhs.rigidBody == second && rhs.rigidBody == first);
}

} // namespace

cTrajectoryPredictor::cTrajectoryPredictor(cSceneRegistry scene, PredictionSettings settings)
    : m_Scene(std::move(scene)), m_Settings(settings) {}

bool cTrajectoryPredictor::IsFinite(const CollisionRequest& request) const {
    const double values[] = {
        request.currentPosition.X, request.currentPosition.Y, request.currentPosition.LAO,
        request.currentPosition.CRAN, request.currentAxles.A1, request.currentAxles.A2,
        request.currentAxles.A3, request.currentAxles.A4, request.direction.x,
        request.direction.y, request.direction.LAO, request.direction.CRAN,
        request.linearSpeedMps, request.angularSpeedRadps
    };
    for (double value : values) if (!std::isfinite(value)) return false;
    return request.session != 0 && request.sequence != 0;
}

bool cTrajectoryPredictor::AreSettingsValid() const {
    const double values[] = {
        m_Settings.reactionTimeS,
        m_Settings.maximumLinearSpeedMps,
        m_Settings.maximumLinearAccelerationMps2,
        m_Settings.guaranteedLinearDecelerationMps2,
        m_Settings.maximumAngularSpeedRadps,
        m_Settings.maximumAngularAccelerationRadps2,
        m_Settings.guaranteedAngularDecelerationRadps2,
        m_Settings.pairMarginM,
        m_Settings.permitLifetimeS,
        m_Settings.intervalMotionToleranceM
    };
    for (double value : values) if (!std::isfinite(value)) return false;
    return m_Settings.reactionTimeS >= 0.0 &&
        m_Settings.maximumLinearSpeedMps >= 0.0 &&
        m_Settings.maximumLinearAccelerationMps2 >= 0.0 &&
        m_Settings.guaranteedLinearDecelerationMps2 > 0.0 &&
        m_Settings.maximumAngularSpeedRadps >= 0.0 &&
        m_Settings.maximumAngularAccelerationRadps2 >= 0.0 &&
        m_Settings.guaranteedAngularDecelerationRadps2 > 0.0 &&
        m_Settings.pairMarginM >= 0.0 &&
        m_Settings.permitLifetimeS > 0.0 &&
        m_Settings.intervalMotionToleranceM >= 0.0 &&
        m_Settings.maximumSubdivisionDepth >= 0;
}

bool cTrajectoryPredictor::IsDirectionValid(const joystickSignal& direction) const {
    const double values[] = {direction.x, direction.y, direction.LAO, direction.CRAN};
    int active = 0;
    for (double value : values) {
        if (value != -1.0 && value != 0.0 && value != 1.0) return false;
        if (value != 0.0) ++active;
    }
    return active == 1;
}

drivePosition cTrajectoryPredictor::PoseAt(const CollisionRequest& request,
                                           double linearTravelM, double angularTravelRad,
                                           double fraction) const {
    const double linearTravelCm = linearTravelM * 100.0;
    const double angularTravelDeg = angularTravelRad * 180.0 / PI;
    return {
        request.currentPosition.X + request.direction.x * linearTravelCm * fraction,
        request.currentPosition.Y + request.direction.y * linearTravelCm * fraction,
        request.currentPosition.LAO + request.direction.LAO * angularTravelDeg * fraction,
        request.currentPosition.CRAN + request.direction.CRAN * angularTravelDeg * fraction
    };
}

bool cTrajectoryPredictor::SolvePose(const drivePosition& position, AxelPostion& axles) const {
    if (position.X < RTMCGeometry::ENVELOPE_MIN_X_CM ||
        position.X > RTMCGeometry::ENVELOPE_MAX_X_CM ||
        position.Y < RTMCGeometry::ENVELOPE_MIN_Y_CM ||
        position.Y > RTMCGeometry::ENVELOPE_MAX_Y_CM) return false;
    cDriveCalculator calculator;
    return calculator.CalculateInverseKinematics(position, axles) == eKinematicStatus::Ok;
}

double cTrajectoryPredictor::FeasibleFraction(const CollisionRequest& request,
                                               double linearTravelM,
                                               double angularTravelRad) const {
    AxelPostion endpoint{};
    if (SolvePose(PoseAt(request, linearTravelM, angularTravelRad, 1.0), endpoint)) return 1.0;
    double valid = 0.0;
    double invalid = 1.0;
    for (int i = 0; i < 48; ++i) {
        const double middle = (valid + invalid) / 2.0;
        AxelPostion candidate{};
        if (SolvePose(PoseAt(request, linearTravelM, angularTravelRad, middle), candidate)) {
            valid = middle;
        } else {
            invalid = middle;
        }
    }
    return valid;
}

bool cTrajectoryPredictor::ShouldCheckPair(const CollisionBody& lhs,
                                           const CollisionBody& rhs) const {
    if (&lhs == &rhs || lhs.rigidBody == rhs.rigidBody) return false;
    if (lhs.frame == eBodyFrame::World && rhs.frame == eBodyFrame::World) return false;
    if (lhs.obstacle || rhs.obstacle) return true;

    // These are the two permanent articulated joints represented by overlapping
    // housing boxes. Other robot pairs remain active for self-collision.
    if (SamePair(lhs, rhs, "link1", "link2")) return false;
    if (SamePair(lhs, rhs, "link2", "carm")) return false;
    return true;
}

cTrajectoryPredictor::eIntervalResult cTrajectoryPredictor::CheckPair(
        const CollisionBody& moving, const CollisionBody& other,
        const AxelPostion& begin, const AxelPostion& middle, const AxelPostion& end,
        CollisionPermit& result) const {
    const OrientedBox movingMiddle = m_Kinematics.CalculateBox(moving, middle);
    const OrientedBox otherMiddle = m_Kinematics.CalculateBox(other, middle);
    const SeparationResult separation = m_Proximity.Separation(movingMiddle, otherMiddle);

    if (separation.overlapping) {
        result.movingBody = moving.id.c_str();
        result.obstacle = other.id.c_str();
        result.reason = "geometry overlaps at a predicted pose";
        return eIntervalResult::Hazard;
    }

    const double movingBound = std::max(m_Kinematics.BoundBodyMotion(moving, middle, begin),
                                        m_Kinematics.BoundBodyMotion(moving, middle, end));
    const double otherBound = std::max(m_Kinematics.BoundBodyMotion(other, middle, begin),
                                       m_Kinematics.BoundBodyMotion(other, middle, end));
    const double requiredGap = m_Settings.pairMarginM + movingBound + otherBound;
    if (separation.largestSeparatingGapM > requiredGap) return eIntervalResult::Clear;

    result.movingBody = moving.id.c_str();
    result.obstacle = other.id.c_str();
    result.reason = "clearance cannot be certified over the prediction interval";
    return eIntervalResult::Unknown;
}

cTrajectoryPredictor::eIntervalResult cTrajectoryPredictor::CheckInterval(
        const CollisionRequest& request, double linearTravelM, double angularTravelRad,
        double begin, double end, CollisionPermit& result) const {
    const auto& bodies = m_Scene.Bodies();
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].frame == eBodyFrame::World) continue;
        for (std::size_t j = 0; j < bodies.size(); ++j) {
            if (!ShouldCheckPair(bodies[i], bodies[j])) continue;
            // Avoid checking the same dynamic/dynamic pair twice.
            if (bodies[j].frame != eBodyFrame::World && j <= i) continue;
            const eIntervalResult pair = CheckPairInterval(
                request, linearTravelM, angularTravelRad, bodies[i], bodies[j],
                begin, end, 0, result);
            if (pair != eIntervalResult::Clear) return pair;
        }
    }
    return eIntervalResult::Clear;
}

cTrajectoryPredictor::eIntervalResult cTrajectoryPredictor::CheckPairInterval(
        const CollisionRequest& request, double linearTravelM, double angularTravelRad,
        const CollisionBody& moving, const CollisionBody& other,
        double begin, double end, int depth, CollisionPermit& result) const {
    const double middleFraction = (begin + end) / 2.0;
    AxelPostion beginAxles{}, middleAxles{}, endAxles{};
    if (!SolvePose(PoseAt(request, linearTravelM, angularTravelRad, begin), beginAxles) ||
        !SolvePose(PoseAt(request, linearTravelM, angularTravelRad, middleFraction), middleAxles) ||
        !SolvePose(PoseAt(request, linearTravelM, angularTravelRad, end), endAxles)) {
        result.reason = "predicted pose is outside the kinematic safety envelope";
        return eIntervalResult::Unknown;
    }
    const eIntervalResult pair = CheckPair(moving, other, beginAxles, middleAxles,
                                           endAxles, result);
    if (pair != eIntervalResult::Unknown) return pair;

    const double movingBound = std::max(
        m_Kinematics.BoundBodyMotion(moving, middleAxles, beginAxles),
        m_Kinematics.BoundBodyMotion(moving, middleAxles, endAxles));
    const double otherBound = std::max(
        m_Kinematics.BoundBodyMotion(other, middleAxles, beginAxles),
        m_Kinematics.BoundBodyMotion(other, middleAxles, endAxles));
    // Once geometry changes by no more than the configured tolerance, further
    // subdivision would claim precision unsupported by the model. Preserve the
    // conservative Unknown result instead of manufacturing a clear permit.
    if (movingBound + otherBound <= m_Settings.intervalMotionToleranceM)
        return eIntervalResult::Unknown;
    if (depth >= m_Settings.maximumSubdivisionDepth) return eIntervalResult::Unknown;

    const eIntervalResult left = CheckPairInterval(request, linearTravelM, angularTravelRad,
        moving, other, begin, middleFraction, depth + 1, result);
    if (left != eIntervalResult::Clear) return left;
    return CheckPairInterval(request, linearTravelM, angularTravelRad,
        moving, other, middleFraction, end, depth + 1, result);
}

CollisionPermit cTrajectoryPredictor::Predict(const CollisionRequest& request) const {
    CollisionPermit result{};
    result.session = request.session;
    result.sequence = request.sequence;
    result.sceneGeneration = m_Scene.Generation();
    result.expiresAt = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(m_Settings.permitLifetimeS));

    if (!m_Scene.IsValid() || request.sceneGeneration != m_Scene.Generation()) {
        result.reason = "scene is invalid or its generation changed";
        return result;
    }
    if (!IsFinite(request) || !IsDirectionValid(request.direction) ||
        !AreSettingsValid()) {
        result.reason = "request or prediction settings are invalid";
        return result;
    }

    result.predictedTravelM = StoppingTravel(request.linearSpeedMps, request.velocityMeasured,
        m_Settings.maximumLinearSpeedMps, m_Settings.maximumLinearAccelerationMps2,
        m_Settings.reactionTimeS, m_Settings.guaranteedLinearDecelerationMps2);
    result.predictedTravelRad = StoppingTravel(request.angularSpeedRadps, request.velocityMeasured,
        m_Settings.maximumAngularSpeedRadps, m_Settings.maximumAngularAccelerationRadps2,
        m_Settings.reactionTimeS, m_Settings.guaranteedAngularDecelerationRadps2);
    const double feasible = FeasibleFraction(request, result.predictedTravelM,
                                             result.predictedTravelRad);

    const eIntervalResult interval = CheckInterval(request, result.predictedTravelM,
                                                   result.predictedTravelRad,
                                                   0.0, feasible, result);
    if (interval == eIntervalResult::Clear) {
        result.verdict = eCollisionVerdict::Clear;
        result.movingBody = "";
        result.obstacle = "";
        result.reason = feasible < 1.0 ? "clear through the reachable stopping path" :
                                        "clear through the predicted stopping path";
    } else if (interval == eIntervalResult::Hazard) {
        result.verdict = eCollisionVerdict::Hazard;
    } else {
        result.verdict = eCollisionVerdict::Unknown;
    }
    return result;
}

} // namespace RTMCCollision
