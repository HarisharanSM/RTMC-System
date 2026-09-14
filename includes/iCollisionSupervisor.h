#pragma once

#include "cCollisionTypes.h"

#include <cstdint>

class iCollisionSupervisor {
public:
    virtual ~iCollisionSupervisor() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual void BeginSession(std::uint64_t session) = 0;
    virtual void AcknowledgeControllerStop(std::uint64_t session) = 0;
    virtual bool Submit(const RTMCCollision::CollisionRequest& request) = 0;
    virtual bool TryConsume(std::uint64_t session, std::uint64_t sequence,
                            RTMCCollision::CollisionPermit& permit) = 0;
    virtual bool IsStopRequested(std::uint64_t session) const = 0;
    virtual std::uint64_t SceneGeneration() const = 0;
    virtual const char* StopReason() const { return "Collision worker revoked motion"; }
};
