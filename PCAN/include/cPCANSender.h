#pragma once
#include "PCANTypes.h"
#include <iostream>

class cPCANSender {
private:
    TPCANHandle m_Channel;
    bool m_IsInitialized;

public:
    cPCANSender(TPCANHandle channel = PCAN_USBBUS1);
    ~cPCANSender();

    bool Initialize(DWORD baudRate = PCAN_BAUD_500K);
    bool Uninitialize();
    
    // Core transmission method
    bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data);
    
    bool IsConnected() const { return m_IsInitialized; }
};