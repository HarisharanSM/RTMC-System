#pragma once
#include "../../includes/PCANTypes.h"
#include "../../includes/commonDrive.h"

class cCANDriveHandler {
public:
    cCANDriveHandler() = default;
    ~cCANDriveHandler() = default;

    /**
     * @brief Translates version-3 or legacy UI CAN direction data into a normalized joystick signal.
     */
    joystickSignal ConvertToJoystickSignal(const TPCANMsg& msg) const;
};
