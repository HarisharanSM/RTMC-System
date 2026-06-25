#include "../include/cCANMocker.h"
#include "../../PCAN/include/cPCANController.h"
#include <iostream>
#include <chrono>
#include <string>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

cCANMocker::cCANMocker(std::shared_ptr<iPCANController> pcanController)
    : m_PcanController(pcanController), m_IsRunning(false) {}

cCANMocker::~cCANMocker() {
    Stop();
}

bool cCANMocker::Start() {
    if (m_IsRunning) return true;
    if (!m_PcanController) return false;

    m_IsRunning = true;
    m_WorkerThread = std::thread(&cCANMocker::MockingLoop, this);
    std::cout << "[cCANMocker] Background bus node simulation initialized.\n";
    return true;
}

void cCANMocker::Stop() {
    if (m_IsRunning) {
        m_IsRunning = false;
        if (m_WorkerThread.joinable()) m_WorkerThread.join();
        std::cout << "[cCANMocker] Bus simulation offline.\n";
    }
}

void cCANMocker::MockingLoop() {
    // Exact requested sequential bit configurations (Bit positions 0 through 7)
    std::map<std::string, int> bitShiftMap = {
        {"R-up",    0}, // Bit 0 -> 1 << 0 = 0x01
        {"R-down",  1}, // Bit 1 -> 1 << 1 = 0x02
        {"R-left",  2}, // Bit 2 -> 1 << 2 = 0x04
        {"R-right", 3}, // Bit 3 -> 1 << 3 = 0x08
        {"L-up",    4}, // Bit 4 -> 1 << 4 = 0x10
        {"L-down",  5}, // Bit 5 -> 1 << 5 = 0x20
        {"L-left",  6}, // Bit 6 -> 1 << 6 = 0x40
        {"L-right", 7}  // Bit 7 -> 1 << 7 = 0x80
    };

    int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8082);

    bind(serverFd, (struct sockaddr*)&address, sizeof(address));
    listen(serverFd, 5);

    std::cout << "[cCANMocker] Bitmask Translator Core Active on Port 8082...\n";

    while (m_IsRunning) {
        int clientSocket = accept(serverFd, nullptr, nullptr);
        if (clientSocket < 0) continue;

        char buffer[1024] = {0};
        int bytesRead = read(clientSocket, buffer, 1024);
        
        if (bytesRead > 0) {
            std::string request(buffer);
            size_t pos = request.find("btn=");
            
            if (pos != std::string::npos) {
                std::string subStr = request.substr(pos + 4);
                size_t spacePos = subStr.find(" ");
                std::string buttonId = (spacePos != std::string::npos) ? subStr.substr(0, spacePos) : subStr;

                if (!buttonId.empty() && buttonId.back() == '\r') buttonId.pop_back();

                // Check if the button identifier exists in our sequence map
                if (bitShiftMap.find(buttonId) != bitShiftMap.end()) {
                    int targetBit = bitShiftMap[buttonId];
                    
                    // Rule: Selected target bit is set to 1, all remaining positions remain 0
                    BYTE bitmaskPayload = static_cast<BYTE>(1 << targetBit);

                    // Build standard TPCANMsg instance
                    TPCANMsg frame{};
                    frame.ID = 0x001;
                    frame.MSGTYPE = PCAN_MESSAGE_STANDARD; // 0x00U
                    frame.LEN = 1;                         // 1 Byte data block length
                    frame.DATA[0] = bitmaskPayload;        // Inject mapped dynamic state value

                    std::cout << "\n[cCANMocker] Web Event '" << buttonId << "' -> Transpiled Bitmask: 0x" 
                              << std::hex << (int)bitmaskPayload << "\n";

                    // Access concrete loopback wrapper logic
                    auto concreteController = std::dynamic_pointer_cast<cPCANController>(m_PcanController);
                    if (concreteController) {
                        // Deliver frame directly to the master receiver endpoint handler instance!
                        concreteController->InjectReceivedMessage(frame);
                    }
                }
            }
        }

        std::string response = 
            "HTTP/1.1 200 OK\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: 0\r\n"
            "Connection: close\r\n\r\n";
        write(clientSocket, response.c_str(), response.length());
        close(clientSocket);
    }
    close(serverFd);
}