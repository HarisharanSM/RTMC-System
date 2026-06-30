#include "../include/cDrive.h"
#include <iostream>

cDrive::cDrive() {
    m_Controller = nullptr;
}

cDrive::~cDrive(){
    Release();
}

bool cDrive::Initialize(std::shared_ptr<iPCANController> pCANptr) {
    std::cout << "[cDrive] Initialising...\n";
    m_Controller = std::make_unique<cDriveController>(pCANptr);
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
}

drivePosition cDrive::GetCurrentPosition() const {
    drivePosition ret = {};
    if (m_Controller) {
        ret = m_Controller->GetCurrentPosition();
    }
    return ret;
}