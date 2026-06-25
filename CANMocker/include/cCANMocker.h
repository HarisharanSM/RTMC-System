#pragma once
#include "../../includes/iPCANController.h"
#include <thread>
#include <atomic>
#include <memory>

class cCANMocker {
private:
    std::shared_ptr<iPCANController> m_PcanController;
    std::thread m_WorkerThread;
    std::atomic<bool> m_IsRunning;

    // Background function that cycles through telemetry values
    void MockingLoop();

public:
    cCANMocker(std::shared_ptr<iPCANController> pcanController);
    ~cCANMocker();

    // Starts the background thread pumping dummy traffic
    bool Start();
    
    // Safely halts the background simulator loop
    void Stop();
};