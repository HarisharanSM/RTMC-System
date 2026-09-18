#pragma once
#include <memory>
#include <functional>
#include "PCANTypes.h"
#include "commonDrive.h"

using MessageCallback = std::function<void(const joystickSignal&)>;

class iPCANController {
public:
    virtual ~iPCANController() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;
    
    virtual void SubscribeMessage(DWORD msgID, MessageCallback callback) = 0;
    virtual bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) = 0;
    virtual void SetSpeed(float speed) = 0;
    virtual void PublishAvoidanceStatus(const char*, const char*, bool = false) {}
    virtual void SetPosition(const AxelPostion& position) = 0;
};
