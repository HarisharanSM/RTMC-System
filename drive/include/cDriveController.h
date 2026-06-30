#pragma once
#include "../../includes/commonDrive.h"
#include "../../includes/iPCANController.h"
#include "cDriveCalculator.h"
#include <iostream>
#include <memory>

class cDriveController {
private:
    bool m_IsEmergencyStopped;
    int m_CurrentErrorCode;
    drivePosition m_CurrentPosition;
    std::shared_ptr<iPCANController> m_pCANController; 
    std::unique_ptr<cDriveCalculator> m_ptrCalculator;

public:
    cDriveController(std::shared_ptr<iPCANController> pCANptr);
    ~cDriveController() = default;

    // Core Control APIs
    void HandleJoystick(const joystickSignal& signal);
    void StartDrive(const joystickSignal& signal);
    void StopDrive(const joystickSignal& signal);
    void SetError(int errorCode);
    void SetEmgStop();

    // Optional Getters for System Monitoring
    bool IsEmergencyStopped() const { return m_IsEmergencyStopped; }
    int GetCurrentErrorCode() const { return m_CurrentErrorCode; }
    drivePosition GetCurrentPosition() const { return m_CurrentPosition; }
};