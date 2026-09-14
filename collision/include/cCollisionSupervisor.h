#pragma once

#include "cSpscMailbox.h"
#include "cTrajectoryPredictor.h"
#include "iCollisionSupervisor.h"

#include <atomic>
#include <cstdint>
#include <thread>

namespace RTMCCollision {

class cCollisionSupervisor : public iCollisionSupervisor {
public:
    explicit cCollisionSupervisor(cSceneRegistry scene,
                                  PredictionSettings settings = PredictionSettings{});
    ~cCollisionSupervisor() override;

    bool Start() override;
    void Stop() override;
    void BeginSession(std::uint64_t session) override;
    void AcknowledgeControllerStop(std::uint64_t session) override;
    bool Submit(const CollisionRequest& request) override;
    bool TryConsume(std::uint64_t session, std::uint64_t sequence,
                    CollisionPermit& permit) override;
    bool IsStopRequested(std::uint64_t session) const override;
    std::uint64_t SceneGeneration() const override;

private:
    void WorkerLoop();
    void RequestStop(std::uint64_t session);

    cTrajectoryPredictor m_Predictor;
    cSpscMailbox<CollisionRequest, 16> m_Requests;
    cSpscMailbox<CollisionPermit, 16> m_Results;
    std::atomic<bool> m_Running{false};
    std::atomic<std::uint64_t> m_ActiveSession{0};
    std::atomic<std::uint64_t> m_StopSession{0};
    std::thread m_Worker;
};

} // namespace RTMCCollision
