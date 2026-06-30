#include "../include/cPCANController.h"
#include <iostream>
#include <cstring>

cPCANController::cPCANController(TPCANHandle channel, DWORD baudRate)
    : m_Channel(channel), m_BaudRate(baudRate), m_IsRunning(false) {
    m_Sender = std::make_unique<cPCANSender>(m_Channel);
    m_Receiver = std::make_unique<cPCANReceiver>(m_Channel);
    m_DriveHandler = std::make_unique<cCANDriveHandler>(); // Initialize
}

cPCANController::~cPCANController() { Stop(); }

bool cPCANController::Start() {
    if (m_IsRunning) return true;
    m_IsRunning = true;
    return true;
}

void cPCANController::Stop() {
    m_IsRunning = false;
    m_SubscriptionRegistry.clear();
}

void cPCANController::SubscribeMessage(DWORD msgID, MessageCallback callback) {
    m_SubscriptionRegistry[msgID] = callback;
    std::cout << "[cPCANController] Registered subscription for Message ID: 0x" << std::hex << msgID << "\n";
}

bool cPCANController::SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) { return true; }

void cPCANController::InjectReceivedMessage(const TPCANMsg& msg) {
    if (!m_IsRunning) return;

    // Convert raw data to joystick structure
    joystickSignal signal{0.0, 0.0, 0.0, 0.0};
    if (msg.ID == DRIVE_MSG || msg.ID == START_DRIVE_MSG || msg.ID == STOP_DRIVE_MSG) {
        signal = m_DriveHandler->ConvertToJoystickSignal(msg);
    }

    std::cout << "[cPCANController] Received Message ID: 0x" << std::hex << msg.ID 
              << " | Data: 0x" << std::hex << (int)msg.DATA[0] << std::dec << "\n";

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
    if (!m_IsRunning) return;

    BYTE dataPayload[8] = {0};
    
    // Serialize float speed profile into the first 4 bytes of our buffer sequence
    std::memcpy(&dataPayload[0], &speed, sizeof(float));

    // Example CAN Target ID for actuator speed configuration registers (e.g., 0x100)
    DWORD speedConfigCanId = 0x100; 
    SendMessage(speedConfigCanId, PCAN_MESSAGE_STANDARD, 4, dataPayload);

    std::cout << "[cPCANController] Hardware CMD -> Outbound Speed Stream: " << speed << " deg/s\n";
}

void cPCANController::SetPosition(const AxelPostion& position) {
    if (!m_IsRunning) return;

    // A standard CAN frame holds up to 8 bytes. Since AxelPostion contains 4 double-precision fields 
    // (4 * 8 bytes = 32 bytes), we serialize these across 4 distinct standard CAN frames.
    
    BYTE frameData[8] = {0};

    // Frame 1: Axis 1 Position
    std::memcpy(frameData, &position.A1, sizeof(double));
    SendMessage(0x201, PCAN_MESSAGE_STANDARD, 8, frameData);

    // Frame 2: Axis 2 Position
    std::memcpy(frameData, &position.A2, sizeof(double));
    SendMessage(0x202, PCAN_MESSAGE_STANDARD, 8, frameData);

    // Frame 3: Axis 3 Position
    std::memcpy(frameData, &position.A3, sizeof(double));
    SendMessage(0x203, PCAN_MESSAGE_STANDARD, 8, frameData);

    // Frame 4: Axis 4 Position
    std::memcpy(frameData, &position.A4, sizeof(double));
    SendMessage(0x204, PCAN_MESSAGE_STANDARD, 8, frameData);

    std::cout << "[cPCANController] Hardware CMD -> Synchronized 4-Axis Position Frames to Bus Topology.\n";
}