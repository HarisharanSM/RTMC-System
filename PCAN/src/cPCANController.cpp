#include "../include/cPCANController.h" 
#include <iostream>

cPCANController::cPCANController(TPCANHandle channel, DWORD baudRate)
    : m_Channel(channel), m_BaudRate(baudRate), m_IsRunning(false) {
    m_Sender = std::make_unique<cPCANSender>(m_Channel);
    m_Receiver = std::make_unique<cPCANReceiver>(m_Channel);
}

cPCANController::~cPCANController() {
    Stop();
}

bool cPCANController::Start(MessageCallback callback) {
    if (m_IsRunning) return true;
    if (!m_Sender->Initialize(m_BaudRate)) return false;
    if (!m_Receiver->Start(callback, m_BaudRate)) {
        m_Sender->Uninitialize();
        return false;
    }
    m_IsRunning = true;
    return true;
}

void cPCANController::Stop() {
    if (!m_IsRunning) return;
    m_Receiver->Stop();
    m_Sender->Uninitialize();
    m_IsRunning = false;
}

bool cPCANController::SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) {
    if (!m_IsRunning) return false;
    return m_Sender->SendMessage(id, msgType, len, data);
}