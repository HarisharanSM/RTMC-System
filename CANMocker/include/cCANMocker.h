#pragma once
#include "iPCANController.h"
#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <cstdint>
class cCANMocker {
    std::shared_ptr<iPCANController> m_PcanController;
    std::thread m_WorkerThread;
    std::atomic<bool> m_IsRunning{false};
    int m_ServerFd = -1;
    std::string m_AssetRoot;
    std::uint16_t m_InputSequence = 0;
    std::uint32_t m_SessionToken = 0;
    std::uint32_t m_NextSessionToken = 0;
    void MockingLoop();
public:
    explicit cCANMocker(std::shared_ptr<iPCANController> controller, std::string assetRoot = ".");
    ~cCANMocker();
    bool Start();
    void Stop();
};
