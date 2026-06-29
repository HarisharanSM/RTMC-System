#pragma once
#include "commonDrive.h"

class iDrive {
public:
    virtual ~iDrive() = default;

    virtual bool Initialize() = 0;
    virtual void HandleJoystick(const joystickSignal& signal) = 0;
    virtual void Release() = 0;
    virtual void SetError(int errorCode) = 0;
    virtual void SetEmgStop() = 0;
    virtual drivePosition GetCurrentPosition() const = 0;
};