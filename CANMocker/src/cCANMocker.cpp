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
    std::map<std::string, int> bitShiftMap = {
        {"R-up",    0}, {"R-down",  1}, {"R-left",  2}, {"R-right", 3},
        {"L-up",    4}, {"L-down",  5}, {"L-left",  6}, {"L-right", 7}
    };

    int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8082);

    if (bind(serverFd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "[cCANMocker] Socket bind failed!\n";
        return;
    }
    listen(serverFd, 5);

    std::cout << "[cCANMocker] Bitmask Translator Core Active on Port 8082...\n";

    while (m_IsRunning) {
        int clientSocket = accept(serverFd, nullptr, nullptr);
        if (clientSocket < 0) continue;

        char buffer[1024] = {0};
        int bytesRead = read(clientSocket, buffer, 1024);
        
        if (bytesRead > 0) {
            std::string request(buffer);
            size_t cmdPos = request.find("cmd=");
            size_t btnPos = request.find("btn=");
            
            TPCANMsg frame{};
            frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
            frame.LEN = 1;
            bool processFrame = false;

            // Check if it's a lifecycle command (start/stop)
            if (cmdPos != std::string::npos) {
                std::string cmdSubStr = request.substr(cmdPos + 4);
                size_t spacePos = cmdSubStr.find(" ");
                size_t ampPos = cmdSubStr.find("&");
                size_t cutPos = (ampPos < spacePos) ? ampPos : spacePos;
                std::string cmdType = (cutPos != std::string::npos) ? cmdSubStr.substr(0, cutPos) : cmdSubStr;

                if (cmdType.find("start") == 0) {
                    frame.ID = 0x002; // StartDrive
                    frame.DATA[0] = 0x01;
                    processFrame = true;
                    std::cout << "[cCANMocker] Internal Command -> Generated StartDrive (0x002)\n";
                } else if (cmdType.find("stop") == 0) {
                    frame.ID = 0x003; // StopDrive
                    frame.DATA[0] = 0x00;
                    processFrame = true;
                    std::cout << "[cCANMocker] Internal Command -> Generated StopDrive (0x003)\n";
                }
            }
            // Fallback: regular continuous signal processing loop
            else if (btnPos != std::string::npos) {
                std::string subStr = request.substr(btnPos + 4);
                size_t spacePos = subStr.find(" ");
                size_t ampPos = subStr.find("&");
                size_t cutPos = (ampPos < spacePos) ? ampPos : spacePos;
                std::string buttonId = (cutPos != std::string::npos) ? subStr.substr(0, cutPos) : subStr;
                
                if (!buttonId.empty() && buttonId.back() == '\r') buttonId.pop_back();

                auto matchIt = bitShiftMap.find(buttonId);
                if (matchIt != bitShiftMap.end()) {
                    BYTE bitmaskPayload = static_cast<BYTE>(1 << matchIt->second);
                    frame.ID = 0x001; // HandleJoystick
                    frame.DATA[0] = bitmaskPayload;
                    processFrame = true;
                    std::cout << "[cCANMocker] Event '" << buttonId << "' -> Bitmask: 0x" 
                              << std::hex << (int)bitmaskPayload << std::dec << "\n";
                }
            }

            if (processFrame) {
                auto concreteController = std::dynamic_pointer_cast<cPCANController>(m_PcanController);
                if (concreteController) {
                    concreteController->InjectReceivedMessage(frame);
                } else {
                    std::cerr << "[cCANMocker] Error: Controller instance is invalid.\n";
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