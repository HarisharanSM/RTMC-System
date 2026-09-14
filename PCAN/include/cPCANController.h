#pragma once
#include "iPCANController.h"
#include "cPCANSender.h"
#include "cPCANReceiver.h"
#include "cCANDriveHandler.h"
#include <map>
#include <mutex>
#include <string>

class cPCANController : public iPCANController {
private:
    mutable std::mutex m_TelemetryMutex;
    AxelPostion m_Axles{-180,180,0,0,0};
    float m_Speed = 0;
    std::uint64_t m_PositionSequence = 0, m_ClearPermits = 0;
    std::string m_State = "Disarmed", m_Reason = "Ready for preflight";
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

    std::string TelemetryJson() const;
    void PublishAvoidanceStatus(const char* state, const char* reason, bool clearPermit = false) override;

    // Simulated hardware operations
    void SetSpeed(float speed) override;
    void SetPosition(const AxelPostion& position) override;
};
