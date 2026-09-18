#pragma once
#include "iPCANController.h"
#include "cPCANSender.h"
#include "cPCANReceiver.h"
#include "cCANDriveHandler.h"
#include <map>
#include <mutex>
#include <string>
#include <chrono>
#include <cstdint>
#include <atomic>
#include <thread>

class cPCANController : public iPCANController {
private:
    mutable std::mutex m_TelemetryMutex;
    std::mutex m_FeedbackTxMutex;
    AxelPostion m_Axles{-180,180,0,0,0};
    float m_Speed = 0;
    float m_CommandedSpeed = 0;
    std::uint64_t m_PositionSequence = 0, m_ClearPermits = 0;
    std::uint32_t m_NextPoseSequence = 0;
    std::uint32_t m_PendingPoseSequence = 0;
    AxelPostion m_PendingAxles{};
    float m_PendingSpeed = 0;
    std::uint8_t m_PendingPoseMask = 0;
    bool m_FeedbackValid = false;
    std::uint64_t m_FeedbackFrameCount = 0;
    std::chrono::steady_clock::time_point m_FeedbackReceivedAt{};
    std::string m_State = "Disarmed", m_Reason = "Ready for preflight";
    TPCANHandle m_Channel;
    DWORD m_BaudRate;
    std::atomic<bool> m_IsRunning{false};
    std::thread m_FeedbackHeartbeat;

    std::unique_ptr<cPCANSender> m_Sender;
    std::unique_ptr<cPCANReceiver> m_Receiver;
    std::unique_ptr<cCANDriveHandler> m_DriveHandler;
    std::map<DWORD, MessageCallback> m_SubscriptionRegistry;

    void ProcessOutboundFeedback(const TPCANMsg& msg);
    bool EmitPoseFeedback(const AxelPostion& position, float speed, std::uint32_t sequence,
                          bool announce = true);
    void FeedbackHeartbeatLoop();

public:
    cPCANController(TPCANHandle channel, DWORD baudRate);
    ~cPCANController() override;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override { return m_IsRunning.load(); }

    void SubscribeMessage(DWORD msgID, MessageCallback callback) override;
    bool SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) override;
    void InjectReceivedMessage(const TPCANMsg& msg);

    std::string TelemetryJson() const;
    void PublishAvoidanceStatus(const char* state, const char* reason, bool clearPermit = false) override;

    // Simulated hardware operations
    void SetSpeed(float speed) override;
    void SetPosition(const AxelPostion& position) override;
};
