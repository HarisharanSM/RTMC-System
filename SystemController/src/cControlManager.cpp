#include "cControlManager.h"
#include "../../PCAN/include/cPCANController.h" 
#include "../../drive/include/cDrive.h"         
#include <iostream>

cControlManager::cControlManager() {
    // Instantiate concrete implementations directly into pure interface pointer storage positions
    m_PcanController = std::make_shared<cPCANController>(PCAN_USBBUS1, PCAN_BAUD_500K);
    m_DriveSubsystem = std::make_unique<cDrive>();
}

cControlManager::~cControlManager() { ShutdownSystem(); }

bool cControlManager::InitializeSystem() {
    std::cout << "[cControlManager] Setting up encapsulated dependency injection stack...\n";
    if (!m_PcanController || !m_DriveSubsystem) return false;

    // 1. Fire up underlying drive components through interface abstract layer APIs
    if (!m_DriveSubsystem->Initialize()) return false;

    // 2. Start communication channel interface layer
    if (!m_PcanController->Start()) return false;

    // 3. Register multi-node interface reference callback binder map records safely
    m_PcanController->SubscribeMessage(
        DRIVE_MSG, 
        std::bind(&iDrive::HandleJoystick, m_DriveSubsystem.get(), std::placeholders::_1)
    );

    return true;
}

void cControlManager::ShutdownSystem() {
    if (m_DriveSubsystem) m_DriveSubsystem->Release();
    if (m_PcanController && m_PcanController->IsRunning()) m_PcanController->Stop();
}

void cControlManager::ProcessUiCommand(int axisId, double velocity) {}