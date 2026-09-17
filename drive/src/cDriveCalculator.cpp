#include "../include/cDriveCalculator.h"

#include <algorithm>
#include <cmath>

using namespace RTMCGeometry;

namespace {

constexpr double PI = 3.14159265358979323846;

inline double ToDegrees(double radians) { return radians * 180.0 / PI; }
inline double ToRadians(double degrees) { return degrees * PI / 180.0; }

inline bool WithinLimit(double value, double lower, double upper, double tolerance) {
    return value >= lower - tolerance && value <= upper + tolerance;
}

inline double MaxJointDelta(const AxelPostion& from, const AxelPostion& to) {
    return std::max({std::abs(to.A1-from.A1), std::abs(to.A2-from.A2),
                     std::abs(to.A3-from.A3), std::abs(to.A4-from.A4), std::abs(to.A5-from.A5)});
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
    clamped.LAO  = ClampTo(pos.LAO, A4_MIN_DEG, A4_MAX_DEG);
    clamped.CRAN = ClampTo(pos.CRAN, A5_MIN_DEG, A5_MAX_DEG);
    return clamped;
}

eKinematicStatus SolveIK(const drivePosition& targetPos, AxelPostion& axelPos, double tolerance) {
    // The two-link chain ends at the column foot. The public X/Y coordinate is
    // the imaging centre, 115 cm along the retained carrier heading.
    const double yaw = ToRadians(targetPos.Yaw);
    const double columnX = targetPos.X + COLUMN_TO_ISOCENTER_CM -
                           COLUMN_TO_ISOCENTER_CM * std::cos(yaw);
    const double columnY = targetPos.Y - COLUMN_TO_ISOCENTER_CM * std::sin(yaw);
    const double x = columnX - BASE_X_CM;
    const double y = columnY - BASE_Y_CM;
    const double distance = std::sqrt(x * x + y * y);

    axelPos.A3 = 0.0;
    axelPos.A4 = targetPos.LAO;
    axelPos.A5 = targetPos.CRAN;

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
    axelPos.A3 = targetPos.Yaw - (axelPos.A1 + axelPos.A2);

    if (!WithinLimit(axelPos.A1, A1_MIN_DEG, A1_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A2, A2_MIN_DEG, A2_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A3, A3_MIN_DEG, A3_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A4, A4_MIN_DEG, A4_MAX_DEG, tolerance) ||
        !WithinLimit(axelPos.A5, A5_MIN_DEG, A5_MAX_DEG, tolerance)) {
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
    const double heading = ToRadians(axelPos.A1 + axelPos.A2 + axelPos.A3);
    const double columnX = BASE_X_CM + LINK1_LEN_CM * std::cos(a1) + LINK2_LEN_CM * std::cos(a12);
    const double columnY = BASE_Y_CM + LINK1_LEN_CM * std::sin(a1) + LINK2_LEN_CM * std::sin(a12);
    pos.X = columnX - COLUMN_TO_ISOCENTER_CM + COLUMN_TO_ISOCENTER_CM * std::cos(heading);
    pos.Y = columnY + COLUMN_TO_ISOCENTER_CM * std::sin(heading);
    pos.LAO = axelPos.A4;
    pos.CRAN = axelPos.A5;
    pos.Yaw = axelPos.A1 + axelPos.A2 + axelPos.A3;
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
    AxelPostion currentAxel{};
    const eKinematicStatus status = ValidatePose(currentPos, currentAxel, GEOM_EPS);
    if (status != eKinematicStatus::Ok) {
        nextPos = currentPos;
        nextAxel = currentAxel;
        return status;
    }
    return CalculateNextPosition(currentPos, currentAxel, signal, nextPos, nextAxel,
                                 deltaTimeMs);
}

eKinematicStatus cDriveCalculator::CalculateNextPosition(const drivePosition& currentPos,
                                                         const AxelPostion& currentAxel,
                                                         const joystickSignal& signal,
                                                         drivePosition& nextPos,
                                                         AxelPostion& nextAxel,
                                                         double deltaTimeMs) {
    const double deltaSeconds = deltaTimeMs / 1000.0;

    // Hold position by default, so every early return still hands back a pose
    // that is safe to commit.
    // The pose we already hold is checked tolerantly - home sits exactly on two
    // joint limits and the inner reach circle.
    AxelPostion solvedCurrent{};
    const eKinematicStatus currentStatus = ValidatePose(currentPos, solvedCurrent, GEOM_EPS);
    nextPos = currentPos;
    nextAxel = currentAxel;

    // Refuse to move from a pose we cannot even solve rather than compounding it.
    if (currentStatus != eKinematicStatus::Ok) {
        return currentStatus;
    }

    const bool idle = signal.x == 0.0 && signal.y == 0.0 &&
                      signal.LAO == 0.0 && signal.CRAN == 0.0 && signal.A3 == 0.0;
    if (idle) {
        ResetMotionProfile();
        return eKinematicStatus::Ok;
    }

    // A3 is a physical carrier rotation, not a virtual compensation value.
    // A1/A2/A4/A5 remain fixed and the public imaging centre follows the
    // resulting 115 cm arc. Exact home is correctly rejected by X >= 0.
    if (signal.A3 != 0.0) {
        m_JointSpeedDps = 0.0;
        if (currentPos.X <= ENVELOPE_MIN_X_CM + GEOM_EPS &&
            std::abs(currentPos.Yaw) <= GEOM_EPS) {
            m_A3SpeedDps = 0.0;
            return eKinematicStatus::OutsideEnvelope;
        }
        m_A3SpeedDps = std::min(MAX_A3_SPEED_DPS,
                                m_A3SpeedDps + A3_ACCEL_DPSS * deltaSeconds);
        const double stepA3 = signal.A3 * m_A3SpeedDps * deltaSeconds;
        const auto evaluateA3 = [&](double fraction, drivePosition& pose,
                                    AxelPostion& axles) {
            axles = currentAxel;
            axles.A3 += fraction * stepA3;
            pose = CalculateForwardKinematics(axles);
            if (!WithinLimit(axles.A3, A3_MIN_DEG, A3_MAX_DEG, 0.0))
                return eKinematicStatus::JointLimit;
            if (!WithinLimit(pose.X, ENVELOPE_MIN_X_CM, ENVELOPE_MAX_X_CM, 0.0) ||
                !WithinLimit(pose.Y, ENVELOPE_MIN_Y_CM, ENVELOPE_MAX_Y_CM, 0.0))
                return eKinematicStatus::OutsideEnvelope;
            return eKinematicStatus::Ok;
        };
        const auto evaluateA3Path = [&](double fraction, drivePosition& pose,
                                        AxelPostion& axles) {
            const eKinematicStatus endpoint = evaluateA3(fraction, pose, axles);
            if (endpoint != eKinematicStatus::Ok) return endpoint;
            const double startHeading = ToRadians(currentAxel.A1 + currentAxel.A2 +
                                                   currentAxel.A3);
            const double headingDelta = ToRadians(stepA3 * fraction);
            if (std::abs(headingDelta) <= GEOM_EPS) return eKinematicStatus::Ok;
            const double lower = std::min(startHeading, startHeading + headingDelta);
            const double upper = std::max(startHeading, startHeading + headingDelta);
            const int first = static_cast<int>(std::ceil(lower / (PI / 2.0)));
            const int last = static_cast<int>(std::floor(upper / (PI / 2.0)));
            for (int k = first; k <= last; ++k) {
                const double crossing = k * PI / 2.0;
                const double pathFraction = (crossing - startHeading) /
                                            ToRadians(stepA3);
                if (pathFraction <= 0.0 || pathFraction >= fraction) continue;
                drivePosition crossingPose{};
                AxelPostion crossingAxles{};
                const eKinematicStatus status = evaluateA3(pathFraction, crossingPose,
                                                            crossingAxles);
                if (status != eKinematicStatus::Ok) return status;
            }
            return eKinematicStatus::Ok;
        };
        drivePosition fullPose{};
        AxelPostion fullAxles{};
        const eKinematicStatus fullStatus = evaluateA3Path(1.0, fullPose, fullAxles);
        if (fullStatus == eKinematicStatus::Ok) {
            nextPos = fullPose;
            nextAxel = fullAxles;
            return eKinematicStatus::Ok;
        }
        double feasible = 0.0, infeasible = 1.0;
        drivePosition feasiblePose = currentPos;
        AxelPostion feasibleAxles = currentAxel;
        for (int i = 0; i < FEASIBILITY_ITERATIONS; ++i) {
            const double middle = 0.5 * (feasible + infeasible);
            drivePosition probePose{};
            AxelPostion probeAxles{};
            if (evaluateA3Path(middle, probePose, probeAxles) == eKinematicStatus::Ok) {
                feasible = middle;
                feasiblePose = probePose;
                feasibleAxles = probeAxles;
            } else {
                infeasible = middle;
            }
        }
        if (feasible < 1e-9) {
            feasiblePose = currentPos;
            feasibleAxles = currentAxel;
        }
        nextPos = feasiblePose;
        nextAxel = feasibleAxles;
        return fullStatus;
    }
    m_A3SpeedDps = 0.0;

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
        probe.Yaw  = currentPos.Yaw;
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
