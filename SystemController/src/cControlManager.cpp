#include "cControlManager.h"
#include "../../PCAN/include/cPCANController.h" 
#include "../../drive/include/cDrive.h"         
#include <iostream>

cControlManager::cControlManager() {
    m_PcanController = std::make_shared<cPCANController>(PCAN_USBBUS1, PCAN_BAUD_500K);
    m_DriveSubsystem = std::make_unique<cDrive>();
}

cControlManager::~cControlManager() { ShutdownSystem(); }

bool cControlManager::InitializeSystem() {
    std::cout << "[cControlManager] Setting up encapsulated dependency injection stack...\n";
    if (!m_PcanController || !m_DriveSubsystem) return false;

    if (!m_DriveSubsystem->Initialize(m_PcanController)) return false; // Match spellings used in your workspace target definitions

    if (!m_PcanController->Start()) return false;

    // Register cControlManager methods as the direct callbacks for each CAN Message ID
    m_PcanController->SubscribeMessage(DRIVE_MSG, std::bind(&iDrive::HandleJoystick, m_DriveSubsystem.get(), std::placeholders::_1));
    m_PcanController->SubscribeMessage(START_DRIVE_MSG, std::bind(&cControlManager::HandleStartDriveSignal, this, std::placeholders::_1));
    m_PcanController->SubscribeMessage(STOP_DRIVE_MSG, std::bind(&cControlManager::HandleStopDriveSignal, this, std::placeholders::_1));

    return true;
}

void cControlManager::HandleStartDriveSignal(const joystickSignal& msg) {
    std::cout << "[cControlManager] Router Callback: Intercepted StartDrive (0x002). Activating motor systems.\n";
    // Your specific logic when the button is first clicked down goes here
    if (m_DriveSubsystem) {
        m_DriveSubsystem->StartDrive(msg);
    }
}

void cControlManager::HandleStopDriveSignal(const joystickSignal& msg) {
    std::cout << "[cControlManager] Router Callback: Intercepted StopDrive (0x003). Bringing axes to rest securely.\n";
    // Your specific logic when the button is released goes here
    if (m_DriveSubsystem) {
        m_DriveSubsystem->StopDrive(msg);
    }
}

void cControlManager::ShutdownSystem() {
    if (m_DriveSubsystem) m_DriveSubsystem->Release();
    if (m_PcanController && m_PcanController->IsRunning()) m_PcanController->Stop();
}

void cControlManager::ProcessUiCommand(int axisId, double velocity) {}