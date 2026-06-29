#pragma once
#include "../../includes/iPCANController.h"
#include "cPCANSender.h"
#include "cPCANReceiver.h"
#include "cCANDriveHandler.h" // Pull in new component
#include <memory>
#include <map>

class cPCANController : public iPCANController {
private:
    TPCANHandle m_Channel;
    DWORD m_BaudRate;
    std::unique_ptr<cPCANSender> m_Sender;
    std::unique_ptr<cPCANReceiver> m_Receiver;
    std::unique_ptr<cCANDriveHandler> m_DriveHandler; // Parser instance
    bool m_IsRunning;

    std::map<DWORD, MessageCallback> m_SubscriptionRegistry;

public:
    cPCANController(TPCANHandle channel = PCAN_USBBUS1, DWORD baudRate = PCAN_BAUD_500K);
    ~cPCANController() override;

    bool Start() override;
    void Stop() override;
    void SubscribeMessage(DWORD msgID, MessageCallback callback) override;
    bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) override;
    bool IsRunning() const override { return m_IsRunning; }

    void InjectReceivedMessage(const TPCANMsg& msg);
};