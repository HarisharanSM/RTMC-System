#pragma once

#define TIME_DELTA  50 //ms

/**
 * @brief Represents the physical position coordinates of the drive system.
 */
struct drivePosition {
    double X;    // Right container horizontal coordinate
    double Y;    // Right container vertical coordinate
    double LAO;  // Left container LAO/RAO angular position
    double CRAN; // Left container CRAN/CAUD angular position
};

/**
 * @brief Represents the processed input states parsed from the joystick matrix.
 */
struct joystickSignal {
    double x;    // Right pad X-axis vector state
    double y;    // Right pad Y-axis vector state
    double LAO;  // Left pad LAO/RAO coordinate value
    double CRAN; // Left pad CRAN/CAUD coordinate value
};

/**
 * @brief Represents the processed output states parsed from the matrix.
 */
struct AxelPostion {
    double A1;    // X,Y
    double A2;    // X,Y
    double A3;  // LAO
    double A4; // CRAN
};