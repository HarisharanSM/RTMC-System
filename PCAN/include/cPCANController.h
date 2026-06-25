#pragma once
#include "../../includes/iPCANController.h"
#include "cPCANSender.h"
#include "cPCANReceiver.h"
#include <memory>

class cPCANController : public iPCANController {
private:
    TPCANHandle m_Channel;
    DWORD m_BaudRate;
    std::unique_ptr<cPCANSender> m_Sender;
    std::unique_ptr<cPCANReceiver> m_Receiver;
    bool m_IsRunning;
    
    // Stored callback reference to forward injected frames
    MessageCallback m_StoredCallback;

public:
    cPCANController(TPCANHandle channel = PCAN_USBBUS1, DWORD baudRate = PCAN_BAUD_500K);
    ~cPCANController() override;

    bool Start(MessageCallback callback) override;
    void Stop() override;
    
    bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) override;
    bool IsRunning() const override { return m_IsRunning; }

    // Re-add this method for the Mocker layer!
    void InjectReceivedMessage(const TPCANMsg& msg);
};