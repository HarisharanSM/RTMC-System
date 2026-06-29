#pragma once
#include "../../includes/iDrive.h"
#include "cDriveController.h" // Include your new controller definition
#include <memory>

class cDrive : public iDrive {
private:
    std::unique_ptr<cDriveController> m_Controller;

public:
    cDrive();
    ~cDrive() override;

    bool Initialize() override;
    void HandleJoystick(const joystickSignal& signal) override;
    void Release() override;
    void SetError(int errorCode) override;
    void SetEmgStop() override;
    drivePosition GetCurrentPosition() const override;
};