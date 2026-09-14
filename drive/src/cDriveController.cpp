#include "../include/cDriveController.h"

#include "cCollisionTypes.h"

#include <chrono>
#include <cmath>

namespace {

constexpr auto PREFLIGHT_DEADLINE = std::chrono::milliseconds(200);

}

cDriveController::cDriveController(std::shared_ptr<iPCANController> pCANptr)
    : cDriveController(std::move(pCANptr), nullptr) {}

cDriveController::cDriveController(
        std::shared_ptr<iPCANController> pCANptr,
        std::unique_ptr<iCollisionSupervisor> collisionSupervisor)
    : m_IsEmergencyStopped(false),
      m_CurrentErrorCode(0),
      m_CurrentPosition{0, 0, 0, 0},
      m_CurrentAxelPosition{0, 0, 0, 0},
      m_LastStatus(eKinematicStatus::Ok),
      m_pCANController(std::move(pCANptr)),
      m_CollisionSupervisor(std::move(collisionSupervisor)) {
    std::cout << "[cDriveController] Internal state engine online.\n";

    m_ptrCalculator = std::make_unique<cDriveCalculator>();

    // Home is world (0,0) - the fully closed pose. Resolve it up front so the
    // reported axle angles match the reported position from the first tick.
    m_ptrCalculator->CalculateInverseKinematics(m_CurrentPosition, m_CurrentAxelPosition);

    if (m_CollisionSupervisor && !m_CollisionSupervisor->Start()) {
        m_CurrentErrorCode = 1001;
        m_LifecycleState.store(eLifecycleState::FaultLatched, std::memory_order_release);
        std::cerr << "[cDriveController] Collision supervisor failed to start.\n";
    } else if (m_CollisionSupervisor) {
        try {
            m_SafetyMonitorRunning.store(true);
            m_SafetyMonitor = std::thread(&cDriveController::SafetyMonitorLoop, this);
        } catch (...) {
            m_SafetyMonitorRunning.store(false);
            m_CollisionSupervisor->Stop();
            m_CurrentErrorCode = 1002;
            m_LifecycleState.store(eLifecycleState::FaultLatched, std::memory_order_release);
            if (m_pCANController) m_pCANController->SetSpeed(0.0f);
            std::cerr << "[cDriveController] Safety monitor failed to start.\n";
        }
    }
}

cDriveController::~cDriveController() {
    m_SafetyMonitorRunning.store(false);
    if (m_SafetyMonitor.joinable()) m_SafetyMonitor.join();
    if (m_CollisionSupervisor) m_CollisionSupervisor->Stop();
}

void cDriveController::SafetyMonitorLoop() {
    while (m_SafetyMonitorRunning.load(std::memory_order_acquire)) {
        const std::uint64_t session = m_Session.load(std::memory_order_acquire);
        const eLifecycleState state = m_LifecycleState.load(std::memory_order_acquire);
        if ((state == eLifecycleState::Preflight || state == eLifecycleState::Running) &&
            m_CollisionSupervisor->IsStopRequested(session)) {
            MonitorStop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void cDriveController::MonitorStop() {
    eLifecycleState state = m_LifecycleState.load(std::memory_order_acquire);
    while (state == eLifecycleState::Preflight || state == eLifecycleState::Running) {
        if (m_LifecycleState.compare_exchange_weak(state, eLifecycleState::AvoidanceLatched,
                                                   std::memory_order_acq_rel)) {
            std::lock_guard<std::mutex> lock(m_CommandMutex);
            if (m_pCANController) m_pCANController->SetSpeed(0.0f);
            std::cerr << "[cDriveController] Predictive avoidance monitor stopped the drive. "
                         "Controller Stop is required before restart.\n";
            return;
        }
    }
}

bool cDriveController::IsSingleDirection(const joystickSignal& signal) {
    const double values[] = {signal.x, signal.y, signal.LAO, signal.CRAN};
    int active = 0;
    for (double value : values) {
        if (!std::isfinite(value) || (value != -1.0 && value != 0.0 && value != 1.0)) return false;
        if (value != 0.0) ++active;
    }
    return active == 1;
}

bool cDriveController::SameDirection(const joystickSignal& lhs, const joystickSignal& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.LAO == rhs.LAO && lhs.CRAN == rhs.CRAN;
}

bool cDriveController::SubmitCollisionRequest() {
    if (!m_CollisionSupervisor) return true;
    RTMCCollision::CollisionRequest request{};
    request.session = m_Session.load(std::memory_order_acquire);
    request.sequence = m_CollisionSequence;
    request.sceneGeneration = m_CollisionSupervisor->SceneGeneration();
    request.currentPosition = m_CurrentPosition;
    request.currentAxles = m_CurrentAxelPosition;
    request.direction = m_ActiveDirection;
    request.linearSpeedMps = MAX_LINEAR_SPEED_CMPS / 100.0;
    request.angularSpeedRadps = MAX_ANGULAR_SPEED_DPS * 3.14159265358979323846 / 180.0;
    request.velocityMeasured = false;
    m_PermitDeadline = std::chrono::steady_clock::now() + PREFLIGHT_DEADLINE;
    return m_CollisionSupervisor->Submit(request);
}

void cDriveController::ProtectiveStop(const char* reason) {
    m_LifecycleState.store(eLifecycleState::AvoidanceLatched, std::memory_order_release);
    if (m_ptrCalculator) m_ptrCalculator->ResetMotionProfile();
    {
        std::lock_guard<std::mutex> lock(m_CommandMutex);
        if (m_pCANController) m_pCANController->SetSpeed(0.0f);
    }
    std::cerr << "[cDriveController] Predictive avoidance stop: " << reason
              << ". Controller Stop is required before restart.\n";
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

    if (m_CollisionSupervisor) {
        const eLifecycleState state = m_LifecycleState.load(std::memory_order_acquire);
        if (state == eLifecycleState::Disarmed || state == eLifecycleState::AvoidanceLatched ||
            state == eLifecycleState::FaultLatched) return;
        if (!SameDirection(signal, m_ActiveDirection)) {
            ProtectiveStop("command direction changed outside the active permit");
            return;
        }
        const std::uint64_t session = m_Session.load(std::memory_order_acquire);
        if (m_CollisionSupervisor->IsStopRequested(session)) {
            ProtectiveStop("collision worker revoked motion");
            return;
        }

        RTMCCollision::CollisionPermit permit{};
        if (!m_CollisionSupervisor->TryConsume(session, m_CollisionSequence, permit)) {
            if (std::chrono::steady_clock::now() > m_PermitDeadline) {
                ProtectiveStop("collision permission deadline expired");
            }
            return;
        }
        if (permit.verdict != RTMCCollision::eCollisionVerdict::Clear ||
            permit.sceneGeneration != m_CollisionSupervisor->SceneGeneration() ||
            std::chrono::steady_clock::now() > permit.expiresAt) {
            ProtectiveStop(permit.reason[0] == '\0' ? "collision permission is invalid" :
                                                     permit.reason);
            return;
        }

        if (state == eLifecycleState::Preflight && m_pCANController) {
            std::lock_guard<std::mutex> lock(m_CommandMutex);
            if (m_LifecycleState.load(std::memory_order_acquire) != eLifecycleState::Preflight ||
                m_CollisionSupervisor->IsStopRequested(session)) return;
            m_pCANController->SetSpeed(static_cast<float>(MAX_JOINT_SPEED_DPS));
        }
        if (!ApplyMotion(signal)) return;
        m_LifecycleState.store(eLifecycleState::Running, std::memory_order_release);
        ++m_CollisionSequence;
        if (!SubmitCollisionRequest()) ProtectiveStop("collision request mailbox is unavailable");
        return;
    }

    ApplyMotion(signal);
}

bool cDriveController::ApplyMotion(const joystickSignal& signal) {

    drivePosition nextPosition{};
    AxelPostion nextAxelPosition{};
    m_LastStatus = m_ptrCalculator->CalculateNextPosition(m_CurrentPosition, signal,
                                                          nextPosition, nextAxelPosition);

    // The calculator only ever hands back a pose that satisfies the envelope,
    // the reach annulus and every joint limit, so position and angles cannot
    // drift apart the way they did when reachability was patched up silently.
    std::unique_lock<std::mutex> commandLock(m_CommandMutex, std::defer_lock);
    if (m_CollisionSupervisor) {
        if (!commandLock.try_lock()) return false;
        const eLifecycleState state = m_LifecycleState.load(std::memory_order_acquire);
        const std::uint64_t session = m_Session.load(std::memory_order_acquire);
        if ((state != eLifecycleState::Preflight && state != eLifecycleState::Running) ||
            m_CollisionSupervisor->IsStopRequested(session)) {
            m_LifecycleState.store(eLifecycleState::AvoidanceLatched, std::memory_order_release);
            m_ptrCalculator->ResetMotionProfile();
            if (m_pCANController) m_pCANController->SetSpeed(0.0f);
            return false;
        }
    }

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
    return true;
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
    if (m_CollisionSupervisor) {
        if (m_LifecycleState.load(std::memory_order_acquire) != eLifecycleState::Disarmed) {
            std::cout << "[cDriveController] Start ignored until controller Stop clears the active session.\n";
            return;
        }
        if (!IsSingleDirection(signal)) {
            std::cerr << "[cDriveController] Cannot start drive: invalid or missing direction.\n";
            return;
        }
        if (m_ptrCalculator) m_ptrCalculator->ResetMotionProfile();
        {
            std::lock_guard<std::mutex> lock(m_CommandMutex);
            if (m_pCANController) m_pCANController->SetSpeed(0.0f);
        }
        std::uint64_t session = m_Session.fetch_add(1, std::memory_order_acq_rel) + 1;
        if (session == 0) {
            session = 1;
            m_Session.store(session, std::memory_order_release);
        }
        m_CollisionSequence = 1;
        m_ActiveDirection = signal;
        m_LifecycleState.store(eLifecycleState::Preflight, std::memory_order_release);
        m_CollisionSupervisor->BeginSession(session);
        if (!SubmitCollisionRequest()) {
            ProtectiveStop("initial collision request could not be queued");
            return;
        }
        std::cout << "[cDriveController] Start accepted; waiting for collision preflight.\n";
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
        std::lock_guard<std::mutex> lock(m_CommandMutex);
        m_pCANController->SetSpeed(0.0f);
    }
    if (m_CollisionSupervisor) {
        m_CollisionSupervisor->AcknowledgeControllerStop(m_Session.load(std::memory_order_acquire));
        m_LifecycleState.store(eLifecycleState::Disarmed, std::memory_order_release);
        m_ActiveDirection = {0, 0, 0, 0};
        m_CollisionSequence = 0;
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
        if (m_CollisionSupervisor) {
            std::lock_guard<std::mutex> lock(m_CommandMutex);
            m_pCANController->SetSpeed(0.0f);
        } else {
            m_pCANController->SetSpeed(0.0f);
        }
    }
    if (m_CollisionSupervisor && errorCode != 0) {
        m_CollisionSupervisor->AcknowledgeControllerStop(m_Session.load(std::memory_order_acquire));
        m_LifecycleState.store(eLifecycleState::FaultLatched, std::memory_order_release);
    }
}

void cDriveController::SetEmgStop() {
    m_IsEmergencyStopped = true;
    std::cerr << "[cDriveController] Emergency flag set. Motion tracks isolated.\n";

    if (m_ptrCalculator) {
        m_ptrCalculator->ResetMotionProfile();
    }
    if (m_pCANController) {
        if (m_CollisionSupervisor) {
            std::lock_guard<std::mutex> lock(m_CommandMutex);
            m_pCANController->SetSpeed(0.0f);
        } else {
            m_pCANController->SetSpeed(0.0f);
        }
    }
    if (m_CollisionSupervisor) {
        m_CollisionSupervisor->AcknowledgeControllerStop(m_Session.load(std::memory_order_acquire));
        m_LifecycleState.store(eLifecycleState::FaultLatched, std::memory_order_release);
    }
}

void cDriveController::ClearEmgStop() {
    m_IsEmergencyStopped = false;
    std::cout << "[cDriveController] Emergency stop released.\n";
}
