#include "cControlManager.h"
#include <iostream>

cControlManager::cControlManager(std::shared_ptr<iPCANController> pcanController)
    : m_PcanController(pcanController) {}

cControlManager::~cControlManager() {
    ShutdownSystem();
}

bool cControlManager::InitializeSystem() {
    std::cout << "[cControlManager] Initializing Main Motion Loop via Interface...\n";
    if (!m_PcanController) return false;

    return m_PcanController->Start([this](const TPCANMsg& msg) { 
        this->OnCanFrameIntercepted(msg); 
    });
}

void cControlManager::ShutdownSystem() {
    if (m_PcanController && m_PcanController->IsRunning()) {
        m_PcanController->Stop();
        std::cout << "[cControlManager] Motion Systems Cleanly Interrupted.\n";
    }
}

void cControlManager::ProcessUiCommand(int axisId, double velocity) {
    std::cout << "[cControlManager] Sending Standard Command Frame...\n";
    BYTE payload[3] = { static_cast<BYTE>(axisId), 0x00, 0x00 };
    m_PcanController->SendMessage(0x200, PCAN_MESSAGE_STANDARD, 3, payload);
}

void cControlManager::OnCanFrameIntercepted(const TPCANMsg& msg) {
    std::cout << "[cControlManager Callback] Packet Received! ID: 0x" 
              << std::hex << msg.ID << " | Type: " << (int)msg.MSGTYPE 
              << " | Data[0]: 0x" << (int)msg.DATA[0] << "\n";
}