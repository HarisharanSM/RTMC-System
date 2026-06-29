#pragma once
#include "iSystemController.h"
#include "iPCANController.h"
#include "iDrive.h"
#include <memory>

class cControlManager : public iSystemController {
private:
    std::shared_ptr<iPCANController> m_PcanController;
    std::unique_ptr<iDrive> m_DriveSubsystem;
    
    // Internal callback route to catch returning system logs or frames
    void OnCanFrameIntercepted(const TPCANMsg& msg);

public:
    // Dependency Injection pattern allows mocking PCAN easily during tests
    cControlManager();
    ~cControlManager() override;

    bool InitializeSystem() override;
    void ShutdownSystem() override;
    void ProcessUiCommand(int axisId, double velocity) override;
    std::shared_ptr<iPCANController> GetCanController() const override { return m_PcanController; }
};