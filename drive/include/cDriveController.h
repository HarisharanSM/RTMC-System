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
    AxelPostion m_CurrentAxelPosition;
    eKinematicStatus m_LastStatus;
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
    void ClearEmgStop();

    // Optional Getters for System Monitoring
    bool IsEmergencyStopped() const { return m_IsEmergencyStopped; }
    int GetCurrentErrorCode() const { return m_CurrentErrorCode; }
    drivePosition GetCurrentPosition() const { return m_CurrentPosition; }

    /** @brief Axle solution for the current position - consistent by construction. */
    AxelPostion GetCurrentAxelPosition() const { return m_CurrentAxelPosition; }

    /** @brief Constraint that bounded the most recent tick, Ok if unconstrained. */
    eKinematicStatus GetLastStatus() const { return m_LastStatus; }

    /** @brief Profile speed in deg/s, for diagnostics and tests. */
    double GetProfileSpeedDps() const {
        return m_ptrCalculator ? m_ptrCalculator->GetProfileSpeedDps() : 0.0;
    }
};
