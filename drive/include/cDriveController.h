#pragma once
#include "../../includes/commonDrive.h"
#include "cDriveCalculator.h"
#include <iostream>
#include <memory>

class cDriveController {
private:
    bool m_IsEmergencyStopped;
    int m_CurrentErrorCode;
    drivePosition m_CurrentPosition;

    std::unique_ptr<cDriveCalculator> m_ptrCalculator; // Pointer of the calculator for kinematic computations

public:
    cDriveController();
    ~cDriveController() = default;

    // Core Control APIs
    void HandleJoystick(const joystickSignal& signal);
    void SetError(int errorCode);
    void SetEmgStop();

    // Optional Getters for System Monitoring
    bool IsEmergencyStopped() const { return m_IsEmergencyStopped; }
    int GetCurrentErrorCode() const { return m_CurrentErrorCode; }
    drivePosition GetCurrentPosition() const { return m_CurrentPosition; }
};