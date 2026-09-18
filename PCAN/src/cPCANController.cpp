#include "../include/cPCANController.h"
#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
void WriteU32(BYTE* data, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) data[i] = static_cast<BYTE>((value >> (8 * i)) & 0xffu);
}
std::uint32_t ReadU32(const BYTE* data) {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(data[i]) << (8 * i);
    return value;
}
void WriteI32(BYTE* data, std::int32_t value) { WriteU32(data, static_cast<std::uint32_t>(value)); }
std::int32_t ReadI32(const BYTE* data) { return static_cast<std::int32_t>(ReadU32(data)); }
constexpr double ANGLE_SCALE = 10000.0;
}

cPCANController::cPCANController(TPCANHandle channel, DWORD baudRate)
    : m_Channel(channel), m_BaudRate(baudRate) {
    m_Sender = std::make_unique<cPCANSender>(m_Channel);
    m_Receiver = std::make_unique<cPCANReceiver>(m_Channel);
    m_DriveHandler = std::make_unique<cCANDriveHandler>(); // Initialize
}

cPCANController::~cPCANController() { Stop(); }

bool cPCANController::Start() {
    if (m_IsRunning.load()) return true;
    if (!m_Sender || !m_Sender->Initialize(m_BaudRate)) return false;
    if (!m_Receiver || !m_Receiver->Start(
            [this](const TPCANMsg& msg) { InjectReceivedMessage(msg); }, m_BaudRate)) {
        m_Sender->Uninitialize();
        return false;
    }
    m_IsRunning.store(true);
    try {
        m_FeedbackHeartbeat = std::thread(&cPCANController::FeedbackHeartbeatLoop, this);
    } catch (...) {
        m_IsRunning.store(false);
        m_Receiver->Stop();
        m_Sender->Uninitialize();
        return false;
    }
    return true;
}

void cPCANController::Stop() {
    m_IsRunning.store(false);
    if (m_FeedbackHeartbeat.joinable()) m_FeedbackHeartbeat.join();
    if (m_Receiver) m_Receiver->Stop();
    if (m_Sender) m_Sender->Uninitialize();
    m_SubscriptionRegistry.clear();
}

void cPCANController::SubscribeMessage(DWORD msgID, MessageCallback callback) {
    m_SubscriptionRegistry[msgID] = callback;
    std::cout << "[cPCANController] Registered subscription for Message ID: 0x" << std::hex << msgID << "\n";
}

bool cPCANController::SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) {
    if (!m_IsRunning.load() || !m_Sender || len > 8) return false;
    if (!m_Sender->SendMessage(id, msgType, len, data)) return false;
    TPCANMsg frame{};
    frame.ID = id;
    frame.MSGTYPE = msgType;
    frame.LEN = len;
    if (data) std::copy(data, data + len, frame.DATA);
    ProcessOutboundFeedback(frame);
    return true;
}

void cPCANController::InjectReceivedMessage(const TPCANMsg& msg) {
    if (!m_IsRunning.load()) return;

    // Convert raw data to joystick structure
    joystickSignal signal{0.0, 0.0, 0.0, 0.0};
    if (msg.ID == DRIVE_MSG || msg.ID == START_DRIVE_MSG || msg.ID == STOP_DRIVE_MSG) {
        signal = m_DriveHandler->ConvertToJoystickSignal(msg);
    }

    // Loop through the registry
    for (const auto& [registeredMsgId, callback] : m_SubscriptionRegistry) {
        if (registeredMsgId == msg.ID) {
            if (callback) {
                callback(signal);
            }
        }
    }
}

void cPCANController::SetSpeed(float speed) {
    if (!m_IsRunning.load()) return;
    AxelPostion currentPosition{};
    std::uint32_t currentSequence = 0;
    bool haveFeedback = false;
    {
        std::lock_guard<std::mutex> lock(m_TelemetryMutex);
        m_CommandedSpeed = speed;
        currentPosition = m_Axles;
        currentSequence = static_cast<std::uint32_t>(m_PositionSequence);
        haveFeedback = m_FeedbackValid;
    }

    BYTE dataPayload[8] = {0};
    
    // Serialize float speed profile into the first 4 bytes of our buffer sequence
    std::memcpy(&dataPayload[0], &speed, sizeof(float));

    // Example CAN Target ID for actuator speed configuration registers (e.g., 0x100)
    DWORD speedConfigCanId = 0x100; 
    SendMessage(speedConfigCanId, PCAN_MESSAGE_STANDARD, 4, dataPayload);
    // A stop or speed-profile change is display-relevant even when position is
    // unchanged. Repeat the current committed pose with fresh CAN speed data.
    if (haveFeedback) EmitPoseFeedback(currentPosition, speed, currentSequence, false);

}

void cPCANController::SetPosition(const AxelPostion& position) {
    if (!m_IsRunning.load()) return;
    const double values[] = {position.A1, position.A2, position.A3, position.A4, position.A5};
    for (double value : values) if (!std::isfinite(value) || std::abs(value) > 360.0) return;

    std::uint32_t sequence;
    float speed;
    {
        std::lock_guard<std::mutex> lock(m_TelemetryMutex);
        sequence = ++m_NextPoseSequence;
        if (sequence == 0) sequence = ++m_NextPoseSequence;
        speed = m_CommandedSpeed;
    }
    // Send fixed-point actuator targets first. The distinct 0x301 feedback
    // sample below is emitted only after every simulated target was accepted.
    BYTE targetPayload[8]{};
    for (int i = 0; i < 5; ++i) {
        WriteU32(targetPayload, sequence);
        WriteI32(targetPayload + 4,
                 static_cast<std::int32_t>(std::llround(values[i] * ANGLE_SCALE)));
        if (!SendMessage(static_cast<DWORD>(0x201 + i), PCAN_MESSAGE_STANDARD,
                         8, targetPayload)) return;
    }
    EmitPoseFeedback(position, speed, sequence);
}

bool cPCANController::EmitPoseFeedback(const AxelPostion& position, float speed,
                                       std::uint32_t sequence, bool announce) {
    std::lock_guard<std::mutex> transmitLock(m_FeedbackTxMutex);
    const double values[] = {position.A1, position.A2, position.A3, position.A4, position.A5};
    BYTE payload[8]{};
    const DWORD ids[] = {DRIVE_POSE_A1_MSG, DRIVE_POSE_A2_MSG, DRIVE_POSE_A3_MSG,
                         DRIVE_POSE_A4_MSG, DRIVE_POSE_A5_MSG};
    for (int i = 0; i < 5; ++i) {
        WriteU32(payload, sequence);
        WriteI32(payload + 4, static_cast<std::int32_t>(std::llround(values[i] * ANGLE_SCALE)));
        if (!SendMessage(ids[i], PCAN_MESSAGE_STANDARD, 8, payload)) return false;
    }
    WriteU32(payload, sequence);
    WriteI32(payload + 4, static_cast<std::int32_t>(std::llround(speed * ANGLE_SCALE)));
    if (!SendMessage(DRIVE_SPEED_MSG, PCAN_MESSAGE_STANDARD, 8, payload)) return false;
    WriteU32(payload, sequence);
    WriteU32(payload + 4, 1); // One simulated controller session token.
    if (!SendMessage(DRIVE_POSE_COMMIT_MSG, PCAN_MESSAGE_STANDARD, 8, payload)) return false;
    (void)announce;
    return true;
}

void cPCANController::FeedbackHeartbeatLoop() {
    while (m_IsRunning.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        AxelPostion position{};
        float speed = 0;
        std::uint32_t sequence = 0;
        {
            std::lock_guard<std::mutex> lock(m_TelemetryMutex);
            if (!m_FeedbackValid) continue;
            position = m_Axles;
            speed = m_CommandedSpeed;
            sequence = static_cast<std::uint32_t>(m_PositionSequence);
        }
        if (m_IsRunning.load()) EmitPoseFeedback(position, speed, sequence, false);
    }
}

void cPCANController::ProcessOutboundFeedback(const TPCANMsg& msg) {
    if (msg.LEN != 8 || msg.ID < DRIVE_POSE_A1_MSG || msg.ID > DRIVE_SPEED_MSG) return;
    const std::uint32_t sequence = ReadU32(msg.DATA);
    if (sequence == 0) return;
    std::lock_guard<std::mutex> lock(m_TelemetryMutex);
    ++m_FeedbackFrameCount;
    if (m_PendingPoseSequence != sequence) {
        m_PendingPoseSequence = sequence;
        m_PendingPoseMask = 0;
        m_PendingAxles = {};
        m_PendingSpeed = 0;
    }
    if (msg.ID >= DRIVE_POSE_A1_MSG && msg.ID <= DRIVE_POSE_A5_MSG) {
        const double angle = static_cast<double>(ReadI32(msg.DATA + 4)) / ANGLE_SCALE;
        if (!std::isfinite(angle) || std::abs(angle) > 360.0) { m_PendingPoseMask = 0; return; }
        double* axes[] = {&m_PendingAxles.A1, &m_PendingAxles.A2, &m_PendingAxles.A3,
                          &m_PendingAxles.A4, &m_PendingAxles.A5};
        *axes[msg.ID - DRIVE_POSE_A1_MSG] = angle;
        m_PendingPoseMask |= static_cast<std::uint8_t>(1u << (msg.ID - DRIVE_POSE_A1_MSG));
    } else if (msg.ID == DRIVE_SPEED_MSG) {
        m_PendingSpeed = static_cast<float>(ReadI32(msg.DATA + 4) / ANGLE_SCALE);
        m_PendingPoseMask |= 0x20u;
    } else if (msg.ID == DRIVE_POSE_COMMIT_MSG) {
        if (m_PendingPoseMask != 0x3fu || sequence < m_PositionSequence) return;
        if (sequence == m_PositionSequence) {
            const bool same = m_PendingAxles.A1 == m_Axles.A1 && m_PendingAxles.A2 == m_Axles.A2 &&
                m_PendingAxles.A3 == m_Axles.A3 && m_PendingAxles.A4 == m_Axles.A4 &&
                m_PendingAxles.A5 == m_Axles.A5;
            if (!same) { m_PendingPoseMask = 0; return; }
            m_Speed = m_PendingSpeed;
            m_FeedbackReceivedAt = std::chrono::steady_clock::now();
            m_PendingPoseMask = 0;
            return;
        }
        m_Axles = m_PendingAxles;
        m_Speed = m_PendingSpeed;
        m_PositionSequence = sequence;
        m_FeedbackReceivedAt = std::chrono::steady_clock::now();
        m_FeedbackValid = true;
        m_PendingPoseMask = 0;
    }
}
void cPCANController::PublishAvoidanceStatus(const char* state, const char* reason, bool clearPermit) {
    std::lock_guard<std::mutex> lock(m_TelemetryMutex);
    m_State = state; m_Reason = reason;
    if (clearPermit) ++m_ClearPermits;
}
std::string cPCANController::TelemetryJson() const {
    std::lock_guard<std::mutex> lock(m_TelemetryMutex);
    const auto age = m_FeedbackValid ? std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - m_FeedbackReceivedAt).count() : -1;
    std::ostringstream out;
    out << std::setprecision(15) << "{\"simulation_only\":true,\"collision_enabled\":true"
        << ",\"protocol_version\":" << RTMC_CAN_PROTOCOL_VERSION
        << ",\"boot_epoch\":1,\"pose_source\":\"drive_can_feedback\""
        << ",\"feedback_valid\":" << (m_FeedbackValid ? "true" : "false")
        << ",\"pose_age_ms\":" << age << ",\"can_feedback_frames\":" << m_FeedbackFrameCount
        << ",\"state\":"
        << std::quoted(m_State) << ",\"reason\":" << std::quoted(m_Reason)
        << ",\"speed_dps\":" << m_Speed << ",\"sequence\":" << m_PositionSequence
        << ",\"pose_sequence\":" << m_PositionSequence
        << ",\"clear_permits\":" << m_ClearPermits << ",\"axles_deg\":["
        << m_Axles.A1 << ',' << m_Axles.A2 << ',' << m_Axles.A3 << ','
        << m_Axles.A4 << ',' << m_Axles.A5 << "]}";
    return out.str();
}
