#pragma once
#include "../../includes/commonDrive.h"
#include "../../includes/iCollisionSupervisor.h"
#include "../../includes/iPCANController.h"
#include "cDriveCalculator.h"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <string>

class cDriveController {
public:
    enum class eLifecycleState {
        Disarmed,
        Preflight,
        Running,
        AvoidanceLatched,
        FaultLatched
    };

private:
    bool m_IsEmergencyStopped;
    int m_CurrentErrorCode;
    drivePosition m_CurrentPosition;
    AxelPostion m_CurrentAxelPosition;
    eKinematicStatus m_LastStatus;
    std::shared_ptr<iPCANController> m_pCANController;
    std::unique_ptr<cDriveCalculator> m_ptrCalculator;
    std::unique_ptr<iCollisionSupervisor> m_CollisionSupervisor;
    std::atomic<eLifecycleState> m_LifecycleState{eLifecycleState::Disarmed};
    joystickSignal m_ActiveDirection{0, 0, 0, 0, 0};
    std::atomic<std::uint64_t> m_Session{0};
    std::uint64_t m_CollisionSequence = 0;
    std::chrono::steady_clock::time_point m_PermitDeadline{};
    std::chrono::steady_clock::time_point m_NextMotionAt{};
    std::atomic<std::int64_t> m_MonitorDeadlineNs{0};
    std::atomic<bool> m_SafetyMonitorRunning{false};
    std::thread m_SafetyMonitor;
    std::mutex m_CommandMutex;

    bool ApplyMotion(const joystickSignal& signal);
    bool SubmitCollisionRequest();
    void ProtectiveStop(const char* reason);
    std::string CollisionStopReason() const;
    void SafetyMonitorLoop();
    void MonitorStop();
    static bool IsSingleDirection(const joystickSignal& signal);
    static bool SameDirection(const joystickSignal& lhs, const joystickSignal& rhs);

public:
    cDriveController(std::shared_ptr<iPCANController> pCANptr);
    cDriveController(std::shared_ptr<iPCANController> pCANptr,
                     std::unique_ptr<iCollisionSupervisor> collisionSupervisor);
    ~cDriveController();

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
    eLifecycleState GetLifecycleState() const { return m_LifecycleState.load(); }
    bool IsAvoidanceLatched() const {
        return m_LifecycleState.load() == eLifecycleState::AvoidanceLatched;
    }

    /** @brief Profile speed in deg/s, for diagnostics and tests. */
    double GetProfileSpeedDps() const {
        return m_ptrCalculator ? m_ptrCalculator->GetProfileSpeedDps() : 0.0;
    }
};
