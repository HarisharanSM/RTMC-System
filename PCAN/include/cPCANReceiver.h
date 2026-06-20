#pragma once
#include "PCANTypes.h"
#include <thread>
#include <atomic>
#include <functional>

class cPCANReceiver {
public:
    // Callback signature for handling inbound motion control signals
    using MessageCallback = std::function<void(const TPCANMsg&)>;

private:
    TPCANHandle m_Channel;
    std::atomic<bool> m_Running;
    std::thread m_ReceiveThread;
    MessageCallback m_Callback;

    void ReceiveLoop();

public:
    cPCANReceiver(TPCANHandle channel = PCAN_USBBUS1);
    ~cPCANReceiver();

    bool Start(MessageCallback callback, DWORD baudRate = PCAN_BAUD_500K);
    void Stop();
};