#include "PCAN/include/cPCANController.h"
#include "cControlManager.h"
#include "cCANMocker.h"
#include <memory>
#include <iostream>
#include <thread>
#include <cstdlib>

int main() {
    std::cout << "=== Native Interfaced Simulation Setup with UI ===\n\n";

    // 1. Core architecture initialization
    auto pcanHardwareHub = std::make_shared<cPCANController>(PCAN_USBBUS1, PCAN_BAUD_500K);
    std::unique_ptr<iSystemController> motionEngine = std::make_unique<cControlManager>(pcanHardwareHub);

    if (!motionEngine->InitializeSystem()) {
        std::cerr << "Initialization failed.\n";
        return -1;
    }

    // 2. Instantiate and launch the native CAN Mocker tool
    cCANMocker busMocker(pcanHardwareHub);
    busMocker.Start();

    // 3. --- Launch Static HTML Web Server ---
    std::cout << "[Main] Spinup requested for local UI directory dashboard assets...\n";
    std::string uiServerCmd = "python3 -m http.server 8000 --directory /workspaces/RTMC-System/ui > /dev/null 2>&1 &";
    
    int sysRet = std::system(uiServerCmd.c_str());
    if (sysRet != 0) {
        std::cout << "[Main] Note: Server call complete.\n";
    }

    // 4. Let the system process the traffic indefinitely
    std::cout << "[Main] System operational. Tracking telemetry execution paths...\n";
    while (true) {
        std::this_thread::sleep_for(std::chrono::hours(24));
    }

    return 0;
}