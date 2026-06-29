#include "../include/cPCANController.h"
#include <iostream>

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
    if (msg.ID == DRIVE_MSG) {
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