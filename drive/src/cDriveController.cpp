#include "../include/cDriveController.h"

cDriveController::cDriveController(std::shared_ptr<iPCANController> pCANptr)
    : m_IsEmergencyStopped(false),
      m_CurrentErrorCode(0),
      m_CurrentPosition{0, 0, 0, 0},
      m_CurrentAxelPosition{0, 0, 0, 0},
      m_LastStatus(eKinematicStatus::Ok),
      m_pCANController(pCANptr) {
    std::cout << "[cDriveController] Internal state engine online.\n";

    m_ptrCalculator = std::make_unique<cDriveCalculator>();

    // Home is world (0,0) - the fully closed pose. Resolve it up front so the
    // reported axle angles match the reported position from the first tick.
    m_ptrCalculator->CalculateInverseKinematics(m_CurrentPosition, m_CurrentAxelPosition);
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
    if (!m_ptrCalculator) {
        return;
    }

    drivePosition nextPosition{};
    AxelPostion nextAxelPosition{};
    m_LastStatus = m_ptrCalculator->CalculateNextPosition(m_CurrentPosition, signal,
                                                          nextPosition, nextAxelPosition);

    // The calculator only ever hands back a pose that satisfies the envelope,
    // the reach annulus and every joint limit, so position and angles cannot
    // drift apart the way they did when reachability was patched up silently.
    m_CurrentPosition = nextPosition;
    m_CurrentAxelPosition = nextAxelPosition;

    if (m_LastStatus != eKinematicStatus::Ok) {
        std::cout << "[cDriveController] Motion constrained by " << ToString(m_LastStatus) << ".\n";
    }

    std::cout << "[cDriveController] Position: X=" << m_CurrentPosition.X
              << ", Y=" << m_CurrentPosition.Y
              << ", LAO=" << m_CurrentPosition.LAO
              << ", CRAN=" << m_CurrentPosition.CRAN << "\n";
    std::cout << "[cDriveController] Axles: A1=" << m_CurrentAxelPosition.A1
              << ", A2=" << m_CurrentAxelPosition.A2
              << ", A3=" << m_CurrentAxelPosition.A3
              << ", A4=" << m_CurrentAxelPosition.A4 << "\n";

    if (m_pCANController) {
        m_pCANController->SetPosition(m_CurrentAxelPosition);
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

    // Motion always begins at the bottom of the acceleration ramp.
    if (m_ptrCalculator) {
        m_ptrCalculator->ResetMotionProfile();
    }
    if (m_pCANController) {
        m_pCANController->SetSpeed(static_cast<float>(MAX_JOINT_SPEED_DPS));
    }
    HandleJoystick(signal);
}

void cDriveController::StopDrive(const joystickSignal& /*signal*/) {
    std::cout << "[cDriveController] Drive stopped successfully.\n";

    if (m_ptrCalculator) {
        m_ptrCalculator->ResetMotionProfile();
    }
    if (m_pCANController) {
        m_pCANController->SetSpeed(0.0f);
    }
}

void cDriveController::SetError(int errorCode) {
    m_CurrentErrorCode = errorCode;
    if (errorCode != 0) {
        std::cerr << "[cDriveController] Active fault register set to code: " << errorCode << "\n";
    } else {
        std::cout << "[cDriveController] Active faults cleared.\n";
    }

    if (m_ptrCalculator) {
        m_ptrCalculator->ResetMotionProfile();
    }
    if (m_pCANController) {
        m_pCANController->SetSpeed(0.0f);
    }
}

void cDriveController::SetEmgStop() {
    m_IsEmergencyStopped = true;
    std::cerr << "[cDriveController] Emergency flag set. Motion tracks isolated.\n";

    if (m_ptrCalculator) {
        m_ptrCalculator->ResetMotionProfile();
    }
    if (m_pCANController) {
        m_pCANController->SetSpeed(0.0f);
    }
}

void cDriveController::ClearEmgStop() {
    m_IsEmergencyStopped = false;
    std::cout << "[cDriveController] Emergency stop released.\n";
}
