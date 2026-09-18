#include "cTrajectoryPredictor.h"

#include "cDriveCalculator.h"

#include <algorithm>
#include <cmath>

namespace RTMCCollision {
namespace {

constexpr double PI = 3.14159265358979323846;

double StoppingTravel(double speed, bool velocityKnown, double maximumSpeed,
                      double acceleration, double reactionTime, double braking) {
    speed = std::abs(speed);
    // The simulator has commanded pose but no measured velocity. Until a motion
    // backend supplies bounded measured feedback, use maximum speed.
    if (!velocityKnown) speed = maximumSpeed;

    double speedAtBrake = speed;
    double reactionTravel = 0.0;
    if (speed < maximumSpeed && acceleration > 0.0) {
        const double accelerationTime = std::min(
            reactionTime, (maximumSpeed - speed) / acceleration);
        speedAtBrake = std::min(maximumSpeed, speed + acceleration * accelerationTime);
        reactionTravel = speed * accelerationTime +
            0.5 * acceleration * accelerationTime * accelerationTime +
            speedAtBrake * (reactionTime - accelerationTime);
    } else if (speed > maximumSpeed) {
        // A lower permit is a ramp-down contract, not an instantaneous clamp.
        // Bound the transition using the same guaranteed deceleration that is
        // used for the final stop.
        const double decelerationTime = std::min(
            reactionTime, (speed - maximumSpeed) / braking);
        speedAtBrake = std::max(maximumSpeed, speed - braking * decelerationTime);
        reactionTravel = speed * decelerationTime -
            0.5 * braking * decelerationTime * decelerationTime +
            speedAtBrake * (reactionTime - decelerationTime);
    } else {
        reactionTravel = speed * reactionTime;
    }
    return reactionTravel + speedAtBrake * speedAtBrake / (2.0 * braking);
}

bool IsAdaptiveAngularMotion(const joystickSignal& direction) {
    return direction.LAO != 0.0 || direction.CRAN != 0.0;
}

} // namespace

cTrajectoryPredictor::cTrajectoryPredictor(cSceneRegistry scene, PredictionSettings settings)
    : m_Scene(std::move(scene)), m_Settings(settings) {}

bool cTrajectoryPredictor::IsFinite(const CollisionRequest& request) const {
    const double values[] = {
        request.currentPosition.X, request.currentPosition.Y, request.currentPosition.LAO,
        request.currentPosition.CRAN, request.currentPosition.Yaw,
        request.currentAxles.A1, request.currentAxles.A2,
        request.currentAxles.A3, request.currentAxles.A4, request.currentAxles.A5, request.direction.x,
        request.direction.y, request.direction.LAO, request.direction.CRAN, request.direction.A3,
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
        m_Settings.maximumA3SpeedRadps,
        m_Settings.maximumA3AccelerationRadps2,
        m_Settings.guaranteedA3DecelerationRadps2,
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
        m_Settings.maximumA3SpeedRadps >= 0.0 &&
        m_Settings.maximumA3AccelerationRadps2 >= 0.0 &&
        m_Settings.guaranteedA3DecelerationRadps2 > 0.0 &&
        m_Settings.pairMarginM >= 0.0 &&
        m_Settings.permitLifetimeS > 0.0 &&
        m_Settings.intervalMotionToleranceM >= 0.0 &&
        m_Settings.maximumSubdivisionDepth >= 0;
}

bool cTrajectoryPredictor::IsDirectionValid(const joystickSignal& direction) const {
    const double values[] = {direction.x, direction.y, direction.LAO, direction.CRAN, direction.A3};
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
        request.currentPosition.CRAN + request.direction.CRAN * angularTravelDeg * fraction,
        request.currentPosition.Yaw
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

bool cTrajectoryPredictor::AxlesAt(const CollisionRequest& request,
                                   double linearTravelM, double angularTravelRad,
                                   double fraction, AxelPostion& axles) const {
    if (request.direction.A3 == 0.0)
        return SolvePose(PoseAt(request, linearTravelM, angularTravelRad, fraction), axles);

    axles = request.currentAxles;
    axles.A3 += request.direction.A3 * angularTravelRad * 180.0 / PI * fraction;
    if (axles.A3 < RTMCGeometry::A3_MIN_DEG || axles.A3 > RTMCGeometry::A3_MAX_DEG)
        return false;
    cDriveCalculator calculator;
    const drivePosition position = calculator.CalculateForwardKinematics(axles);
    return position.X >= RTMCGeometry::ENVELOPE_MIN_X_CM &&
           position.X <= RTMCGeometry::ENVELOPE_MAX_X_CM &&
           position.Y >= RTMCGeometry::ENVELOPE_MIN_Y_CM &&
           position.Y <= RTMCGeometry::ENVELOPE_MAX_Y_CM;
}

double cTrajectoryPredictor::FeasibleFraction(const CollisionRequest& request,
                                               double linearTravelM,
                                               double angularTravelRad) const {
    if (request.direction.A3 != 0.0) {
        AxelPostion endpoint{};
        if (!AxlesAt(request, linearTravelM, angularTravelRad, 1.0, endpoint))
            return -1.0;
        const double startHeading = (request.currentAxles.A1 + request.currentAxles.A2 +
                                     request.currentAxles.A3) * PI / 180.0;
        const double headingDelta = request.direction.A3 * angularTravelRad;
        if (std::abs(headingDelta) <= 1e-15) return 1.0;
        const double lower = std::min(startHeading, startHeading + headingDelta);
        const double upper = std::max(startHeading, startHeading + headingDelta);
        const int first = static_cast<int>(std::ceil(lower / (PI / 2.0)));
        const int last = static_cast<int>(std::floor(upper / (PI / 2.0)));
        for (int k = first; k <= last; ++k) {
            const double crossing = k * PI / 2.0;
            const double fraction = (crossing - startHeading) / headingDelta;
            AxelPostion crossingAxles{};
            if (fraction > 0.0 && fraction < 1.0 &&
                !AxlesAt(request, linearTravelM, angularTravelRad, fraction,
                          crossingAxles)) return -1.0;
        }
        return 1.0;
    }
    AxelPostion endpoint{};
    if (AxlesAt(request, linearTravelM, angularTravelRad, 1.0, endpoint)) return 1.0;
    // A3 must be able to execute its complete bounded stop. Clipping the check
    // at a joint/workspace boundary would issue a permit without proving that
    // the rotating carrier can actually stop before that boundary.
    double valid = 0.0;
    double invalid = 1.0;
    for (int i = 0; i < 48; ++i) {
        const double middle = (valid + invalid) / 2.0;
        AxelPostion candidate{};
        if (AxlesAt(request, linearTravelM, angularTravelRad, middle, candidate)) {
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

    // Synthetic joint-interface boxes intentionally overlap at their bearings.
    // The exclusions are generated from the same source as the body geometry.
    if (m_Scene.IsPairExcluded(lhs, rhs)) return false;
    if (lhs.obstacle || rhs.obstacle) return true;
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
    // SAT's largest axis gap is a conservative lower bound. Spend the more
    // expensive closest-feature calculation only for near, diagonal cases.
    if (m_Proximity.SurfaceDistance(movingMiddle, otherMiddle) > requiredGap)
        return eIntervalResult::Clear;

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
    if (!AxlesAt(request, linearTravelM, angularTravelRad, begin, beginAxles) ||
        !AxlesAt(request, linearTravelM, angularTravelRad, middleFraction, middleAxles) ||
        !AxlesAt(request, linearTravelM, angularTravelRad, end, endAxles)) {
        result.reason = "predicted pose is outside the kinematic safety envelope";
        return eIntervalResult::Unknown;
    }
    // Bound joint excursions across the entire Cartesian interval, not just
    // endpoint differences: a joint can reverse while X or Y stays monotone.
    // q1=atan2(y,x)-alpha(r), q2=acos(c(r)). For L2>L1 alpha and q2
    // decrease with radius. Radius extrema include the segment's projection
    // onto the base, while atan2 is monotone on this single-axis path (x>0).
    if (request.direction.x != 0.0 || request.direction.y != 0.0) {
        const auto p0=PoseAt(request,linearTravelM,angularTravelRad,begin);
        const auto p1=PoseAt(request,linearTravelM,angularTravelRad,end);
        const double yaw=p0.Yaw*PI/180.0;
        const double offsetX=RTMCGeometry::COLUMN_TO_ISOCENTER_CM*(1.0-std::cos(yaw));
        const double offsetY=-RTMCGeometry::COLUMN_TO_ISOCENTER_CM*std::sin(yaw);
        const double x0=p0.X+offsetX-RTMCGeometry::BASE_X_CM, y0=p0.Y+offsetY-RTMCGeometry::BASE_Y_CM;
        const double x1=p1.X+offsetX-RTMCGeometry::BASE_X_CM, y1=p1.Y+offsetY-RTMCGeometry::BASE_Y_CM;
        const double dx=x1-x0, dy=y1-y0, length2=dx*dx+dy*dy;
        const double projection=length2>0 ? std::clamp(-(x0*dx+y0*dy)/length2,0.0,1.0) : 0.0;
        const double rMin=std::hypot(x0+projection*dx,y0+projection*dy);
        const double rMax=std::max(std::hypot(x0,y0),std::hypot(x1,y1));
        const double l1=RTMCGeometry::LINK1_LEN_CM, l2=RTMCGeometry::LINK2_LEN_CM;
        const auto alpha=[&](double r) {return std::acos(std::clamp((l1*l1+r*r-l2*l2)/(2*l1*r),-1.0,1.0))*180/PI;};
        const auto elbow=[&](double r) {return std::acos(std::clamp((r*r-l1*l1-l2*l2)/(2*l1*l2),-1.0,1.0))*180/PI;};
        const double theta0=std::atan2(y0,x0)*180/PI, theta1=std::atan2(y1,x1)*180/PI;
        const double q1Min=std::min(theta0,theta1)-alpha(rMin);
        const double q1Max=std::max(theta0,theta1)-alpha(rMax);
        const double dq1=std::max(std::abs(q1Min-middleAxles.A1),std::abs(q1Max-middleAxles.A1));
        const double dq2=std::max(std::abs(elbow(rMin)-middleAxles.A2),std::abs(elbow(rMax)-middleAxles.A2));
        beginAxles.A1=middleAxles.A1-dq1; endAxles.A1=middleAxles.A1+dq1;
        beginAxles.A2=middleAxles.A2-dq2; endAxles.A2=middleAxles.A2+dq2;
        beginAxles.A3=p0.Yaw-(beginAxles.A1+beginAxles.A2);
        endAxles.A3=p0.Yaw-(endAxles.A1+endAxles.A2);
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
    if (!m_Scene.IsValid() || request.sceneGeneration != m_Scene.Generation()) {
        result.reason = "scene is invalid or its generation changed";
        return result;
    }

    if (!IsFinite(request) || !IsDirectionValid(request.direction) ||
        !AreSettingsValid()) {
        result.reason = "request or prediction settings are invalid";
        return result;
    }
    result.expiresAt = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(m_Settings.permitLifetimeS));

    result.predictedTravelM = StoppingTravel(request.linearSpeedMps, request.velocityMeasured,
        m_Settings.maximumLinearSpeedMps, m_Settings.maximumLinearAccelerationMps2,
        m_Settings.reactionTimeS, m_Settings.guaranteedLinearDecelerationMps2);
    const bool angularVelocityKnown = request.velocityMeasured || request.velocityModeled;
    if (request.direction.A3 != 0.0) {
        result.predictedTravelRad = StoppingTravel(request.angularSpeedRadps, angularVelocityKnown,
            m_Settings.maximumA3SpeedRadps, m_Settings.maximumA3AccelerationRadps2,
            m_Settings.reactionTimeS, m_Settings.guaranteedA3DecelerationRadps2);
        result.permittedAngularSpeedRadps = m_Settings.maximumA3SpeedRadps;
    }

    double angularCandidates[] = {
        m_Settings.maximumAngularSpeedRadps,
        m_Settings.maximumAngularSpeedRadps * 0.5,
        m_Settings.maximumAngularSpeedRadps * 0.25,
        10.0 * PI / 180.0,
        5.0 * PI / 180.0
    };
    for (double& candidate : angularCandidates)
        candidate = std::min(candidate, m_Settings.maximumAngularSpeedRadps);
    std::sort(std::begin(angularCandidates), std::end(angularCandidates),
              [](double lhs, double rhs) { return lhs > rhs; });
    const int candidateCount = IsAdaptiveAngularMotion(request.direction) &&
                               angularVelocityKnown ? 5 : 1;
    CollisionPermit denial = result;
    for (int candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex) {
        CollisionPermit candidate = result;
        double angularLimit = request.direction.A3 != 0.0 ?
            m_Settings.maximumA3SpeedRadps : angularCandidates[candidateIndex];
        candidate.permittedAngularSpeedRadps = angularLimit;
        if (request.direction.A3 == 0.0) {
            candidate.predictedTravelRad = StoppingTravel(
                request.angularSpeedRadps, angularVelocityKnown, angularLimit,
                m_Settings.maximumAngularAccelerationRadps2,
                m_Settings.reactionTimeS,
                m_Settings.guaranteedAngularDecelerationRadps2);
        }

        const double feasible = FeasibleFraction(request, candidate.predictedTravelM,
                                                 candidate.predictedTravelRad);
        if (feasible < 0.0) {
            candidate.reason = "A3 stopping path exceeds a joint or workspace limit";
            denial = candidate;
            continue;
        }
        const eIntervalResult interval = CheckInterval(
            request, candidate.predictedTravelM, candidate.predictedTravelRad,
            0.0, feasible, candidate);
        if (interval == eIntervalResult::Clear) {
            candidate.verdict = eCollisionVerdict::Clear;
            candidate.movingBody = "";
            candidate.obstacle = "";
            candidate.reason = IsAdaptiveAngularMotion(request.direction) &&
                               angularLimit + 1e-12 < m_Settings.maximumAngularSpeedRadps ?
                "clear with collision-limited angular speed" :
                (feasible < 1.0 ? "clear through the reachable stopping path" :
                                  "clear through the predicted stopping path");
            return candidate;
        }
        candidate.verdict = interval == eIntervalResult::Hazard ?
            eCollisionVerdict::Hazard : eCollisionVerdict::Unknown;
        denial = candidate;
    }
    return denial;
}

} // namespace RTMCCollision
