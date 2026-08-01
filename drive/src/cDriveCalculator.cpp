#include "../include/cDriveCalculator.h"

#include <algorithm>
#include <cmath>

using namespace RTMCGeometry;

namespace {

constexpr double PI = 3.14159265358979323846;

inline double ToDegrees(double radians) { return radians * 180.0 / PI; }

inline bool WithinLimit(double value, double lower, double upper, double tolerance) {
    return value >= lower - tolerance && value <= upper + tolerance;
}

inline double MaxJointDelta(const AxelPostion& from, const AxelPostion& to) {
    return std::max(std::max(std::abs(to.A1 - from.A1), std::abs(to.A2 - from.A2)),
                    std::max(std::abs(to.A3 - from.A3), std::abs(to.A4 - from.A4)));
}

// Number of bisection steps used to find the largest feasible fraction of a
// requested move. 40 halvings take the bracket below 1e-12 of the step.
constexpr int FEASIBILITY_ITERATIONS = 40;

// Relative slack on the per-tick angular budget. An axis asked to move exactly
// its budget must be allowed to, so the comparison cannot be exact - but the
// slack is scaled to the budget rather than being a flat GEOM_EPS, which would
// let every tick overshoot by a fixed absolute amount.
constexpr double RATE_TOLERANCE = 1e-9;

inline double ClampTo(double value, double lower, double upper) {
    return std::min(std::max(value, lower), upper);
}

// Belt and braces on the commit path: candidate steps are already validated
// with zero tolerance, so this only ever trims floating-point dust.
inline drivePosition ClampToLimits(const drivePosition& pos) {
    drivePosition clamped = pos;
    clamped.X    = ClampTo(pos.X, ENVELOPE_MIN_X_CM, ENVELOPE_MAX_X_CM);
    clamped.Y    = ClampTo(pos.Y, ENVELOPE_MIN_Y_CM, ENVELOPE_MAX_Y_CM);
    clamped.LAO  = ClampTo(pos.LAO, A3_MIN_DEG, A3_MAX_DEG);
    clamped.CRAN = ClampTo(pos.CRAN, A4_MIN_DEG, A4_MAX_DEG);
    return clamped;
}

eKinematicStatus SolveIK(const drivePosition& targetPos, AxelPostion& axelPos, double tolerance) {
    // Target expressed in the base frame.
    const double x = targetPos.X - BASE_X_CM;
    const double y = targetPos.Y - BASE_Y_CM;
    const double distance = std::sqrt(x * x + y * y);

    // A3/A4 are pass-through axes; report them even if the planar solve fails.
    axelPos.A3 = targetPos.LAO;
    axelPos.A4 = targetPos.CRAN;

    if (distance > MAX_REACH_CM + tolerance) {
        axelPos.A1 = 0.0;
        axelPos.A2 = 0.0;
        return eKinematicStatus::OutOfReach;
    }
    if (distance < MIN_REACH_CM - tolerance) {
        axelPos.A1 = 0.0;
        axelPos.A2 = 0.0;
        return eKinematicStatus::TooClose;
    }

    // cos(A2) from the law of cosines. At either reach boundary this is exactly
    // -1 or +1, so it is clamped before the sqrt below - a single ulp past the
    // boundary would otherwise yield NaN and put NaN on the bus.
    double cosA2 = (x * x + y * y - LINK1_LEN_CM * LINK1_LEN_CM - LINK2_LEN_CM * LINK2_LEN_CM) /
                   (2.0 * LINK1_LEN_CM * LINK2_LEN_CM);
    cosA2 = std::clamp(cosA2, -1.0, 1.0);

    // Positive root keeps A2 in [0, 180]: 180 is fully closed, 0 fully extended.
    const double a2 = std::atan2(std::sqrt(1.0 - cosA2 * cosA2), cosA2);
    const double a1 = std::atan2(y, x) -
                      std::atan2(LINK2_LEN_CM * std::sin(a2),
                                 LINK1_LEN_CM + LINK2_LEN_CM * std::cos(a2));

    axelPos.A1 = ToDegrees(a1);
    axelPos.A2 = ToDegrees(a2);

    if (!WithinLimit(axelPos.A1, A1_MIN_DEG, A1_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A2, A2_MIN_DEG, A2_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A3, A3_MIN_DEG, A3_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A4, A4_MIN_DEG, A4_MAX_DEG, tolerance)) {
        return eKinematicStatus::JointLimit;
    }

    return eKinematicStatus::Ok;
}

} // namespace

eKinematicStatus cDriveCalculator::CalculateInverseKinematics(const drivePosition& targetPos,
                                                              AxelPostion& axelPos) const {
    return SolveIK(targetPos, axelPos, GEOM_EPS);
}

drivePosition cDriveCalculator::CalculateForwardKinematics(const AxelPostion& axelPos) const {
    const double a1 = axelPos.A1 * PI / 180.0;
    // A2 is relative to link 1, so link 2's absolute heading is A1 + A2.
    const double a12 = (axelPos.A1 + axelPos.A2) * PI / 180.0;

    drivePosition pos{};
    pos.X = BASE_X_CM + LINK1_LEN_CM * std::cos(a1) + LINK2_LEN_CM * std::cos(a12);
    pos.Y = BASE_Y_CM + LINK1_LEN_CM * std::sin(a1) + LINK2_LEN_CM * std::sin(a12);
    pos.LAO = axelPos.A3;
    pos.CRAN = axelPos.A4;
    return pos;
}

eKinematicStatus cDriveCalculator::ValidatePose(const drivePosition& pos, AxelPostion& axelPos,
                                                double tolerance) const {
    const eKinematicStatus solved = SolveIK(pos, axelPos, tolerance);

    if (!WithinLimit(pos.X, ENVELOPE_MIN_X_CM, ENVELOPE_MAX_X_CM, tolerance) ||
        !WithinLimit(pos.Y, ENVELOPE_MIN_Y_CM, ENVELOPE_MAX_Y_CM, tolerance)) {
        return eKinematicStatus::OutsideEnvelope;
    }

    return solved;
}

eKinematicStatus cDriveCalculator::CalculateNextPosition(const drivePosition& currentPos,
                                                         const joystickSignal& signal,
                                                         drivePosition& nextPos,
                                                         AxelPostion& nextAxel,
                                                         double deltaTimeMs) {
    const double deltaSeconds = deltaTimeMs / 1000.0;

    // Hold position by default, so every early return still hands back a pose
    // that is safe to commit.
    // The pose we already hold is checked tolerantly - home sits exactly on two
    // joint limits and the inner reach circle.
    AxelPostion currentAxel{};
    const eKinematicStatus currentStatus = ValidatePose(currentPos, currentAxel, GEOM_EPS);
    nextPos = currentPos;
    nextAxel = currentAxel;

    // Refuse to move from a pose we cannot even solve rather than compounding it.
    if (currentStatus != eKinematicStatus::Ok) {
        return currentStatus;
    }

    const bool idle = signal.x == 0.0 && signal.y == 0.0 &&
                      signal.LAO == 0.0 && signal.CRAN == 0.0;
    if (idle) {
        ResetMotionProfile();
        return eKinematicStatus::Ok;
    }

    // Trapezoidal profile: ramp the per-axle speed, then derive this tick's
    // angular budget from it.
    m_JointSpeedDps = std::min(MAX_JOINT_SPEED_DPS,
                               m_JointSpeedDps + JOINT_ACCEL_DPSS * deltaSeconds);
    const double jointBudgetDeg = m_JointSpeedDps * deltaSeconds;

    // Requested step, in the units each axis is actually measured in.
    const double stepX    = signal.x    * MAX_LINEAR_SPEED_CMPS * deltaSeconds;
    const double stepY    = signal.y    * MAX_LINEAR_SPEED_CMPS * deltaSeconds;
    const double stepLAO  = signal.LAO  * MAX_ANGULAR_SPEED_DPS * deltaSeconds;
    const double stepCRAN = signal.CRAN * MAX_ANGULAR_SPEED_DPS * deltaSeconds;

    const auto poseAtFraction = [&](double fraction) {
        drivePosition probe{};
        probe.X    = currentPos.X    + fraction * stepX;
        probe.Y    = currentPos.Y    + fraction * stepY;
        probe.LAO  = currentPos.LAO  + fraction * stepLAO;
        probe.CRAN = currentPos.CRAN + fraction * stepCRAN;
        return probe;
    };

    // A fraction is feasible when the pose is legal and no axle has to move
    // further than the profile allows in one tick.
    // Candidate steps are checked with zero tolerance - see ValidatePose.
    const auto evaluate = [&](double fraction, AxelPostion& axelOut) {
        const eKinematicStatus status = ValidatePose(poseAtFraction(fraction), axelOut, 0.0);
        if (status != eKinematicStatus::Ok) {
            return status;
        }
        if (MaxJointDelta(currentAxel, axelOut) > jointBudgetDeg * (1.0 + RATE_TOLERANCE)) {
            return eKinematicStatus::RateLimited;
        }
        return eKinematicStatus::Ok;
    };

    // Snap onto the exact limits and re-solve, so a committed pose is never
    // GEOM_EPS outside the envelope or an axle's travel. Clamping only ever
    // moves inward; if that somehow fails to solve, keep the validated pose.
    const auto commit = [&](double fraction, const AxelPostion& validatedAxel) {
        const drivePosition clamped = ClampToLimits(poseAtFraction(fraction));
        AxelPostion clampedAxel{};
        if (ValidatePose(clamped, clampedAxel, GEOM_EPS) == eKinematicStatus::Ok) {
            nextPos = clamped;
            nextAxel = clampedAxel;
        } else {
            nextPos = poseAtFraction(fraction);
            nextAxel = validatedAxel;
        }
    };

    AxelPostion fullAxel{};
    const eKinematicStatus fullStatus = evaluate(1.0, fullAxel);
    if (fullStatus == eKinematicStatus::Ok) {
        commit(1.0, fullAxel);
        return eKinematicStatus::Ok;
    }

    // Largest feasible fraction of the requested step. Only accepted probes are
    // ever promoted, so the committed pose is feasible even where the constraint
    // boundary is not perfectly monotone in the fraction.
    double feasible = 0.0;
    double infeasible = 1.0;
    AxelPostion feasibleAxel = currentAxel;
    for (int i = 0; i < FEASIBILITY_ITERATIONS; ++i) {
        const double middle = 0.5 * (feasible + infeasible);
        AxelPostion probeAxel{};
        if (evaluate(middle, probeAxel) == eKinematicStatus::Ok) {
            feasible = middle;
            feasibleAxel = probeAxel;
        } else {
            infeasible = middle;
        }
    }

    commit(feasible, feasibleAxel);
    return fullStatus;
}
