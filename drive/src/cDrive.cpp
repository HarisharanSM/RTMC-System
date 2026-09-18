#include "../include/cDrive.h"
#include "cCollisionSupervisor.h"
#include "cSceneRegistry.h"
#include <iostream>

cDrive::cDrive() {
    m_Controller = nullptr;
}

cDrive::~cDrive(){
    Release();
}

bool cDrive::Initialize(std::shared_ptr<iPCANController> pCANptr) {
    std::cout << "[cDrive] Initialising...\n";
    auto supervisor = std::make_unique<RTMCCollision::cCollisionSupervisor>(
        RTMCCollision::cSceneRegistry::CreateReferenceScene(true));
    m_Controller = std::make_unique<cDriveController>(pCANptr, std::move(supervisor));
    if (m_Controller->GetCurrentErrorCode() != 0) return false;
    // Publish home through the drive's sequenced CAN feedback path before the
    // UI connects. Direct controller unit fixtures keep their original counts.
    if (pCANptr) pCANptr->SetPosition(m_Controller->GetCurrentAxelPosition());
    return true;
}

void cDrive::HandleJoystick(const joystickSignal& signal) {
    if (m_Controller) m_Controller->HandleJoystick(signal);
}

void cDrive::StartDrive(const joystickSignal& signal) {
    if (m_Controller) m_Controller->StartDrive(signal);
}

void cDrive::StopDrive(const joystickSignal& signal) {
    if (m_Controller) m_Controller->StopDrive(signal);
}

void cDrive::SetError(int errorCode) {
    if (m_Controller) m_Controller->SetError(errorCode);
}

void cDrive::SetEmgStop() {
    if (m_Controller) m_Controller->SetEmgStop();
}

void cDrive::Release() {
    std::cout << "[cDrive] Released.\n";
    m_Controller.reset();
}

drivePosition cDrive::GetCurrentPosition() const {
    drivePosition ret = {};
    if (m_Controller) {
        ret = m_Controller->GetCurrentPosition();
    }
    return ret;
}
