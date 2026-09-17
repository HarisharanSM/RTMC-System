#pragma once
#include "commonDrive.h"

/**
 * Machine geometry and axle travel.
 *
 * These describe the physical positioner. Replace the joint limits with the
 * real values from the machine data sheet - the ones below are the ranges the
 * current link geometry needs in order to cover the declared envelope.
 */
namespace RTMCGeometry {

inline constexpr double LINK1_LEN_CM = 75.0;   // base axle -> elbow
inline constexpr double LINK2_LEN_CM = 100.0;  // elbow -> C-arm mount

inline constexpr double MIN_REACH_CM = LINK2_LEN_CM - LINK1_LEN_CM;  //  25 - fully closed
inline constexpr double MAX_REACH_CM = LINK1_LEN_CM + LINK2_LEN_CM;  // 175 - fully extended

/**
 * Base origin, derived rather than chosen.
 *
 * World (0,0) must be the fully-closed pose. When closed (A2 = 180 deg) the end
 * effector sits MIN_REACH_CM from the base along A1 + 180 deg. Picking
 * A1_home = 180 deg so the arm opens toward +X puts the base at (-MIN_REACH, 0),
 * which also keeps the base behind the whole travel range so the arm never has
 * to fold through it.
 */
inline constexpr double BASE_X_CM = -MIN_REACH_CM;
inline constexpr double BASE_Y_CM = 0.0;
inline constexpr double COLUMN_TO_ISOCENTER_CM = 115.0;

// Axle travel. A1 = -180 / A2 = 180 is the closed pose; A1 = A2 = 0 is full
// extension. A1's positive limit covers the upper-corner geometry.
inline constexpr double A1_MIN_DEG = -180.0;
inline constexpr double A1_MAX_DEG =   10.0;
inline constexpr double A2_MIN_DEG =    0.0;
inline constexpr double A2_MAX_DEG =  180.0;
inline constexpr double A3_MIN_DEG = -180.0;
inline constexpr double A3_MAX_DEG =  180.0;
inline constexpr double A4_MIN_DEG = -180.0;
inline constexpr double A4_MAX_DEG =  180.0;
// CRAN/CAUD is mechanically limited to 90 degrees in either direction.
inline constexpr double A5_MIN_DEG =  -90.0;
inline constexpr double A5_MAX_DEG =   90.0;

// Declared safety envelope, kept consistent with the reach annulus: at Y = 0 the
// arm reaches exactly BASE_X + MAX_REACH = 150 cm.
inline constexpr double ENVELOPE_MIN_X_CM =   0.0;
inline constexpr double ENVELOPE_MAX_X_CM = BASE_X_CM + MAX_REACH_CM;
inline constexpr double ENVELOPE_MIN_Y_CM = -25.0;
inline constexpr double ENVELOPE_MAX_Y_CM =  25.0;

// Tolerance for boundary comparisons. The home pose sits exactly on both the
// inner reach circle and two joint limits, so exact comparisons would reject it.
inline constexpr double GEOM_EPS = 1e-6;

} // namespace RTMCGeometry

/**
 * @brief Kinematics and motion profiling for the two-axle planar positioner.
 *
 * Angle convention: A2 is measured relative to link 1, so the absolute heading
 * of link 2 is A1 + A2. CalculateForwardKinematics is the exact inverse of
 * CalculateInverseKinematics under that convention.
 */
class cDriveCalculator {
public:
    cDriveCalculator() = default;
    ~cDriveCalculator() = default;

    /**
     * @brief Solve the axle angles for a target pose.
     *
     * Never rescales or retargets: an unreachable request is reported, not
     * quietly replaced by a nearby one. @p axelPos is populated with the
     * solution whenever the geometry admits one, even if that solution violates
     * a joint limit, so callers can inspect why it was refused.
     *
     * Does not apply the safety envelope - that is a motion-level concern.
     */
    eKinematicStatus CalculateInverseKinematics(const drivePosition& targetPos,
                                                AxelPostion& axelPos) const;

    /** @brief End-effector pose for a set of axle angles. Inverse of the above. */
    drivePosition CalculateForwardKinematics(const AxelPostion& axelPos) const;

    /**
     * @brief Advance one control tick.
     *
     * Builds the requested step from the joystick direction and the commanded
     * speeds, then shortens it until it satisfies the envelope, the reach
     * annulus, every joint limit and the per-tick joint speed budget.
     *
     * @param nextPos   accepted pose - always valid and safe to commit
     * @param nextAxel  axle solution for @p nextPos, consistent by construction
     * @return Ok if the full step was taken, otherwise the binding constraint
     */
    eKinematicStatus CalculateNextPosition(const drivePosition& currentPos,
                                           const joystickSignal& signal,
                                           drivePosition& nextPos,
                                           AxelPostion& nextAxel,
                                           double deltaTimeMs = TIME_DELTA_MS);

    /** Advance one tick while preserving the independently driven A3 state. */
    eKinematicStatus CalculateNextPosition(const drivePosition& currentPos,
                                           const AxelPostion& currentAxel,
                                           const joystickSignal& signal,
                                           drivePosition& nextPos,
                                           AxelPostion& nextAxel,
                                           double deltaTimeMs = TIME_DELTA_MS);

    /** @brief Return the trapezoidal profile to rest. Call on stop/fault/e-stop. */
    void ResetMotionProfile() { m_JointSpeedDps = 0.0; m_A3SpeedDps = 0.0; }

    /** @brief Current profile speed in deg/s, for diagnostics and tests. */
    double GetProfileSpeedDps() const { return m_JointSpeedDps; }

private:
    /**
     * @brief Envelope + reach + joint-limit check for a candidate pose.
     *
     * @param tolerance slack on every limit comparison. The pose we are already
     *        holding is checked with GEOM_EPS, because the home pose sits
     *        exactly on two joint limits and the inner reach circle and would
     *        otherwise fail to validate. Candidate steps are checked with zero
     *        tolerance: slack may accept a pose, but it must never authorise
     *        motion, or a saturated axis renews that slack every tick and the
     *        remaining axes creep by one tolerance per tick without bound.
     */
    eKinematicStatus ValidatePose(const drivePosition& pos, AxelPostion& axelPos,
                                  double tolerance) const;

    double m_JointSpeedDps = 0.0;  // trapezoidal profile state, deg/s
    double m_A3SpeedDps = 0.0;     // independent A3 profile state, deg/s
};
