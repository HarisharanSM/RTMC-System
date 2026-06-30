#include "../include/cDriveController.h"

cDriveController::cDriveController(std::shared_ptr<iPCANController> pCANptr)
    : m_IsEmergencyStopped(false), 
      m_CurrentErrorCode(0), 
      m_CurrentPosition{0,0,0,0},
      m_pCANController(pCANptr) {
    std::cout << "[cDriveController] Internal state engine online.\n";
    
    m_ptrCalculator = std::make_unique<cDriveCalculator>();
}

void cDriveController::HandleJoystick(const joystickSignal& signal) {
    // Safety verification check bounds
    if (m_IsEmergencyStopped) {
        std::cout << "[cDriveController] Blocked: Axis is locked under emergency stop.\n";
        return;
    }
    if (m_CurrentErrorCode != 0) {
        std::cout << "[cDriveController] Blocked: Clear active error code " << m_CurrentErrorCode << " first.\n";
        return;
    }

    // Move your calculation logic here (e.g., updating position metrics)
    std::cout << "[cDriveController] Executing kinematics matrix calculation updates...\n";
    
    if(m_ptrCalculator){
        drivePosition nextPosition = m_ptrCalculator->CalculateNextPosition(m_CurrentPosition, signal);

        AxelPostion axelPos;
        m_ptrCalculator->CalculateInverseKinematics(nextPosition, axelPos);

        // Update the current position after successful calculation
        m_CurrentPosition = nextPosition;

        std::cout << "[cDriveController] Updated current position: X=" << m_CurrentPosition.X 
                  << ", Y=" << m_CurrentPosition.Y 
                  << ", LAO=" << m_CurrentPosition.LAO 
                  << ", CRAN=" << m_CurrentPosition.CRAN << "\n";
        std::cout << "[cDriveController] Calculated axel positions: A1=" << axelPos.A1 
                  << ", A2=" << axelPos.A2 
                  << ", A3=" << axelPos.A3 
                  << ", A4=" << axelPos.A4 << "\n";

        if(m_pCANController){
            m_pCANController->SetPosition(axelPos);
        }
    }
}

void cDriveController::StartDrive(const joystickSignal& signal) {
    if (m_IsEmergencyStopped) {
        std::cout << "[cDriveController] Cannot start drive: Emergency stop is active.\n";
        return;
    }
    if (m_CurrentErrorCode != 0) {
        std::cout << "[cDriveController] Cannot start drive: Active error code " << m_CurrentErrorCode << " must be cleared first.\n";
        return;
    }
    std::cout << "[cDriveController] Drive started successfully.\n";
    
    if(m_pCANController){
        m_pCANController->SetSpeed(MAX_SPEED); 
    }
    HandleJoystick(signal);
}

void cDriveController::StopDrive(const joystickSignal& signal) {
    std::cout << "[cDriveController] Drive stopped successfully.\n";
    
    if(m_pCANController){
        m_pCANController->SetSpeed(0.0f); 
    }
    HandleJoystick(signal);
}

void cDriveController::SetError(int errorCode) {
    m_CurrentErrorCode = errorCode;
    if (errorCode != 0) {
        std::cerr << "[cDriveController] Active fault register set to code: " << errorCode << "\n";
    } else {
        std::cout << "[cDriveController] Active faults cleared.\n";
    }

    if(m_pCANController){
        m_pCANController->SetSpeed(0.0f); 
    }
}

void cDriveController::SetEmgStop() {
    m_IsEmergencyStopped = true;
    std::cerr << "[cDriveController] Emergency flag set. Motion tracks isolated.\n";

    if(m_pCANController){
        m_pCANController->SetSpeed(0.0f); 
    }
}