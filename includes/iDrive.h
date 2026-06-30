#pragma once
#include "commonDrive.h"
#include <memory>
#include "iPCANController.h"

class iDrive {
public:
    virtual ~iDrive() = default;

    virtual bool Initialize(std::shared_ptr<iPCANController> pCANptr) = 0;
    virtual void HandleJoystick(const joystickSignal& signal) = 0;
    virtual void StartDrive(const joystickSignal& signal) = 0;
    virtual void StopDrive(const joystickSignal& signal) = 0;
    virtual void Release() = 0;
    virtual void SetError(int errorCode) = 0;
    virtual void SetEmgStop() = 0;
    virtual drivePosition GetCurrentPosition() const = 0;
};