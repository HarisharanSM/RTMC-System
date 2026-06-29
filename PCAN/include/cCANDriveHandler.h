#pragma once
#include "../../includes/PCANTypes.h"
#include "../../includes/commonDrive.h"

class cCANDriveHandler {
public:
    cCANDriveHandler() = default;
    ~cCANDriveHandler() = default;

    /**
     * @brief Translates an incoming 8-bit sequential CAN frame into a normalized joystickSignal structural layout.
     */
    joystickSignal ConvertToJoystickSignal(const TPCANMsg& msg) const;
};