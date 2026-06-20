#include "PCAN/include/cPCANController.h"  // Located in PCAN/src/
#include "cControlManager.h"   // Located in SystemController/include/
#include <memory>
#include <iostream>
#include <chrono>

int main() {
    std::cout << "=== Advanced Interfaced Control Architecture ===\n\n";

    // 1. Instantiate concrete communication engine layer
    auto pcanHardwareHub = std::make_shared<cPCANController>(PCAN_USBBUS1, PCAN_BAUD_500K);

    // 2. Inject dependency straight down into our high level systems abstraction context manager
    std::unique_ptr<iSystemController> motionEngine = std::make_unique<cControlManager>(pcanHardwareHub);

    // 3. Initialize pipeline safely
    if (!motionEngine->InitializeSystem()) {
        std::cerr << "Initialization configuration collapsed.\n";
        return -1;
    }

    // 4. Exercise application loops using simulated UI mutations
    motionEngine->ProcessUiCommand(1, 1500.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    motionEngine->ProcessUiCommand(2, -750.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 5. Cleanup runtime resources
    motionEngine->ShutdownSystem();
    return 0;
}