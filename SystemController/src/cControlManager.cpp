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

    // Direct binding of internal worker logic to PCAN pipeline stream
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
    std::cout << "[cControlManager] Translating High-level UI Input to CAN Payload...\n";
    
    BYTE payload[8] = {0};
    payload[0] = static_cast<BYTE>(axisId);
    // Dummy compression format mapping velocity data onto primitive byte structures
    payload[1] = (static_cast<int>(velocity) >> 8) & 0xFF; 
    payload[2] = static_cast<int>(velocity) & 0xFF;

    // Execute via interface virtual table safely
    m_PcanController->SendMessage(0x200, PCAN_MESSAGE_STANDARD, 3, payload);
}

void cControlManager::OnCanFrameIntercepted(const TPCANMsg& msg) {
    std::cout << "[cControlManager Callback] Incoming Driver State Checked. ID: 0x" 
              << std::hex << msg.ID << " | Frame Type: " << (int)msg.MSGTYPE << "\n";
}