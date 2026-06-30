#pragma once
#include "iPCANController.h"
#include "cPCANSender.h"
#include "cPCANReceiver.h"
#include "cCANDriveHandler.h"
#include <map>

class cPCANController : public iPCANController {
private:
    TPCANHandle m_Channel;
    DWORD m_BaudRate;
    bool m_IsRunning;

    std::unique_ptr<cPCANSender> m_Sender;
    std::unique_ptr<cPCANReceiver> m_Receiver;
    std::unique_ptr<cCANDriveHandler> m_DriveHandler;
    std::map<DWORD, MessageCallback> m_SubscriptionRegistry;

public:
    cPCANController(TPCANHandle channel, DWORD baudRate);
    ~cPCANController() override;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override { return m_IsRunning; }

    void SubscribeMessage(DWORD msgID, MessageCallback callback) override;
    bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) override;
    void InjectReceivedMessage(const TPCANMsg& msg);

    // ✅ New Overrides for Hardware Operations
    void SetSpeed(float speed) override;
    void SetPosition(const AxelPostion& position) override;
};