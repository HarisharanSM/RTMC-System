#pragma once
#include "PCANTypes.h"
#include "commonDrive.h"
#include <functional>

class iPCANController {
public:
    using MessageCallback = std::function<void(const joystickSignal&)>;

    virtual ~iPCANController() = default;
    
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual void SubscribeMessage(DWORD msgID, MessageCallback callback) = 0;
    
    // Abstract Sender API wrapper
    virtual bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) = 0;
    virtual bool IsRunning() const = 0;
};