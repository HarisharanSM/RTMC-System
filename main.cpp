#include "SystemController/include/cControlManager.h"
#include "CANMocker/include/cCANMocker.h"
#include <memory>
#include <iostream>
#include <thread>
#include <cstdlib>

int main() {
    std::cout << "=== Interface Isolated System Initialization Setup ===\n\n";

    // 1. Instantiate the single high-level systems engine loop wrapper block
    std::unique_ptr<iSystemController> motionEngine = std::make_unique<cControlManager>();

    if (!motionEngine->InitializeSystem()) {
        std::cerr << "Initialization failed.\n";
        return -1;
    }

    std::cout << "[Main] Booting HTTP CAN Mocker Translation Engine...\n";
    auto sharedPcan = motionEngine->GetCanController();
    
    auto canMocker = std::make_unique<cCANMocker>(sharedPcan);
    canMocker->Start();

    // 2. --- Launch Static HTML Web Server for Remote View Dashboard ---
    std::cout << "[Main] Spinning up local UI asset dashboard directories...\n";
    std::string uiServerCmd = "python3 -m http.server 8000 --directory /workspaces/RTMC-System/ui > /dev/null 2>&1 &";
    std::system(uiServerCmd.c_str());

    std::cout << "[Main] System online. Operational tracks routing cleanly via abstract layers.\n";
    while (true) {
        std::this_thread::sleep_for(std::chrono::hours(24));
    }
    std::cout << "[Main] System offline.\n";
    return 0;
}