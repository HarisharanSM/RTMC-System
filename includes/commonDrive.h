#pragma once

// ---------------------------------------------------------------------------
// Control loop timing
// ---------------------------------------------------------------------------
inline constexpr double TIME_DELTA_MS = 50.0;   // one control tick, in ms

// ---------------------------------------------------------------------------
// Motion limits.
//
// The joint limits describe what the axles themselves can do; the commanded
// limits describe how fast the operator is allowed to request tool motion. The
// joint profile is always the authority - a Cartesian request is shortened
// until every axle fits inside its per-tick budget.
// ---------------------------------------------------------------------------
inline constexpr double MAX_JOINT_SPEED_DPS   = 60.0;   // per-axle speed,        deg/s
inline constexpr double JOINT_ACCEL_DPSS      = 120.0;  // per-axle acceleration, deg/s^2
inline constexpr double MAX_LINEAR_SPEED_CMPS = 20.0;   // commanded X/Y speed,   cm/s
inline constexpr double MAX_ANGULAR_SPEED_DPS = 60.0;   // commanded LAO/CRAN,    deg/s

/**
 * @brief Represents the physical position coordinates of the drive system.
 *
 * (X, Y) is the end-effector position in the world frame, in cm. The origin is
 * fixed at the initial patient head; A1/A2 are closed at that origin.
 */
struct drivePosition {
    double X;    // End-effector horizontal coordinate, cm
    double Y;    // End-effector transverse coordinate, cm
    double LAO;  // LAO/RAO angular position, deg
    double CRAN; // CRAN/CAUD angular position, deg
};

/**
 * @brief Represents the processed input states parsed from the joystick matrix.
 *
 * Each field is a direction only: -1, 0 or +1. Magnitude comes from the speed
 * constants above, never from the signal itself.
 */
struct joystickSignal {
    double x;    // Right pad X-axis direction
    double y;    // Right pad Y-axis direction
    double LAO;  // Left pad LAO/RAO direction
    double CRAN; // Left pad CRAN/CAUD direction
};

/**
 * @brief Axle angles, in degrees.
 *
 * A1 is the base axle. A2 is carried on A1's hand end and is measured RELATIVE
 * to link 1 (the textbook 2R elbow angle), not in the world frame: the absolute
 * heading of link 2 is A1 + A2. A3 cancels that heading; A4/A5 are LAO/CRAN about the imaging center.
 */
struct AxelPostion {
    double A1;   // Base axle,               deg
    double A2;   // Elbow axle, relative,    deg
    double A3;   // Mount yaw compensation: -(A1+A2), deg
    double A4;   // LAO/RAO, deg
    double A5 = 0.0; // CRAN/CAUD, deg
};

/**
 * @brief Outcome of a kinematic request.
 *
 * Anything other than Ok means the request was constrained. The pose handed
 * back by cDriveCalculator is always valid and committable regardless of the
 * status - the status explains which constraint shortened the motion.
 */
enum class eKinematicStatus {
    Ok,               // fully satisfied, unconstrained
    OutsideEnvelope,  // outside the declared safety envelope
    OutOfReach,       // further away than L1 + L2
    TooClose,         // inside the folded dead zone |L2 - L1|
    JointLimit,       // solvable, but an axle would exceed its travel
    RateLimited       // shortened to respect the joint speed profile
};

inline const char* ToString(eKinematicStatus status) {
    switch (status) {
        case eKinematicStatus::Ok:              return "Ok";
        case eKinematicStatus::OutsideEnvelope: return "OutsideEnvelope";
        case eKinematicStatus::OutOfReach:      return "OutOfReach";
        case eKinematicStatus::TooClose:        return "TooClose";
        case eKinematicStatus::JointLimit:      return "JointLimit";
        case eKinematicStatus::RateLimited:     return "RateLimited";
    }
    return "Unknown";
}
