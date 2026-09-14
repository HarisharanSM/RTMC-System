#include "cCollisionSupervisor.h"

#include <chrono>
#include <thread>

namespace RTMCCollision {

cCollisionSupervisor::cCollisionSupervisor(cSceneRegistry scene, PredictionSettings settings)
    : m_Predictor(std::move(scene), settings) {}

cCollisionSupervisor::~cCollisionSupervisor() { Stop(); }

bool cCollisionSupervisor::Start() {
    bool expected = false;
    if (!m_Running.compare_exchange_strong(expected, true)) return true;
    try {
        m_Worker = std::thread(&cCollisionSupervisor::WorkerLoop, this);
    } catch (...) {
        m_Running.store(false);
        return false;
    }
    return true;
}

void cCollisionSupervisor::Stop() {
    if (!m_Running.exchange(false)) return;
    if (m_Worker.joinable()) m_Worker.join();
    m_ActiveSession.store(0, std::memory_order_release);
    m_StopSession.store(0, std::memory_order_release);
    m_Requests.Drain();
    m_Results.Drain();
}

void cCollisionSupervisor::BeginSession(std::uint64_t session) {
    m_Results.Drain();
    m_StopSession.store(0, std::memory_order_release);
    m_ActiveSession.store(session, std::memory_order_release);
}

void cCollisionSupervisor::AcknowledgeControllerStop(std::uint64_t session) {
    std::uint64_t expected = session;
    m_ActiveSession.compare_exchange_strong(expected, 0, std::memory_order_acq_rel);
    if (m_StopSession.load(std::memory_order_acquire) == session) {
        m_StopSession.store(0, std::memory_order_release);
    }
    m_Results.Drain();
}

bool cCollisionSupervisor::Submit(const CollisionRequest& request) {
    if (!m_Running.load(std::memory_order_acquire) ||
        request.session != m_ActiveSession.load(std::memory_order_acquire)) return false;
    if (!m_Requests.Push(request)) {
        RequestStop(request.session);
        return false;
    }
    return true;
}

bool cCollisionSupervisor::TryConsume(std::uint64_t session, std::uint64_t sequence,
                                      CollisionPermit& permit) {
    CollisionPermit candidate{};
    while (m_Results.Pop(candidate)) {
        if (candidate.session != session || candidate.sequence < sequence) continue;
        if (candidate.sequence > sequence) {
            RequestStop(session);
            return false;
        }
        permit = candidate;
        return true;
    }
    return false;
}

bool cCollisionSupervisor::IsStopRequested(std::uint64_t session) const {
    return session != 0 && m_StopSession.load(std::memory_order_acquire) == session;
}

std::uint64_t cCollisionSupervisor::SceneGeneration() const {
    return m_Predictor.Scene().Generation();
}

void cCollisionSupervisor::RequestStop(std::uint64_t session) {
    if (session != 0 && session == m_ActiveSession.load(std::memory_order_acquire)) {
        m_StopSession.store(session, std::memory_order_release);
    }
}

void cCollisionSupervisor::WorkerLoop() {
    while (m_Running.load(std::memory_order_acquire)) {
        CollisionRequest request{};
        if (!m_Requests.Pop(request)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (request.session != m_ActiveSession.load(std::memory_order_acquire)) continue;

        CollisionPermit result = m_Predictor.Predict(request);
        if (request.session != m_ActiveSession.load(std::memory_order_acquire)) continue;
        if (result.verdict != eCollisionVerdict::Clear) {
            m_StopReason.store(result.reason, std::memory_order_release);
            RequestStop(request.session);
        }
        if (!m_Results.Push(result)) RequestStop(request.session);
    }
}

} // namespace RTMCCollision
