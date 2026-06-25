#include "cPCANReceiver.h"
#include <iostream>
#include <chrono>

cPCANReceiver::cPCANReceiver(TPCANHandle channel) 
    : m_Channel(channel), m_Running(false) {}

cPCANReceiver::~cPCANReceiver() {
    Stop();
}

bool cPCANReceiver::Start(MessageCallback callback, DWORD baudRate) {
    if (m_Running) return false;
    
    m_Callback = callback;
    m_Running = true;
    
    // In production: PCAN initialization happens here.
    std::cout << "[cPCANReceiver] Subscribed to channel " << (int)m_Channel << "\n";
    
    // Spawn background worker loop to poll for incoming CAN messages
    m_ReceiveThread = std::thread(&cPCANReceiver::ReceiveLoop, this);
    return true;
}

void cPCANReceiver::Stop() {
    if (m_Running) {
        m_Running = false;
        if (m_ReceiveThread.joinable()) {
            m_ReceiveThread.join();
        }
        std::cout << "[cPCANReceiver] Stopped listening on channel " << (int)m_Channel << "\n";
    }
}

void cPCANReceiver::ReceiveLoop() {
    // Keep the thread alive, but strip out the automatic dummy message generator
    while (m_Running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        
        /* On production hardware, real frames from physical controllers 
           would be captured here via CAN_Read() and sent to m_Callback.
           For our cloud setup, we leave this empty so only our UI Mocker 
           triggers data prints.
        */
    }
}