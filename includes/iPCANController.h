#pragma once
#include "PCANTypes.h"
#include <functional>

class iPCANController {
public:
    using MessageCallback = std::function<void(const TPCANMsg&)>;

    virtual ~iPCANController() = default;
    
    virtual bool Start(MessageCallback callback) = 0;
    virtual void Stop() = 0;
    
    // Abstract Sender API wrapper
    virtual bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) = 0;
    virtual bool IsRunning() const = 0;
};