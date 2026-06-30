#pragma once
#include "../../includes/iDrive.h"
#include "../../includes/iPCANController.h"
#include "cDriveController.h" // Include your new controller definition
#include <memory>

class cDrive : public iDrive {
private:
    std::unique_ptr<cDriveController> m_Controller;

public:
    cDrive();
    ~cDrive() override;

    bool Initialize(std::shared_ptr<iPCANController> pCANptr) override;
    void HandleJoystick(const joystickSignal& signal) override;
    void StartDrive(const joystickSignal& signal) override;
    void StopDrive(const joystickSignal& signal) override;
    void Release() override;
    void SetError(int errorCode) override;
    void SetEmgStop() override;
    drivePosition GetCurrentPosition() const override;
};