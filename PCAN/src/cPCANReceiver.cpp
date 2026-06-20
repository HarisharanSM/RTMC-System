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
    while (m_Running) {
        TPCANMsg receivedMsg;
        bool messageAvailable = false;

        /* Production Implementation:
        DWORD result = CAN_Read(m_Channel, &receivedMsg, NULL);
        if (result == PCAN_ERROR_OK) {
            messageAvailable = true;
        }
        */

        // Simple Mock Simulation for codespaces environment loop
        std::this_thread::sleep_for(std::chrono::milliseconds(800)); 
        static DWORD mockCounter = 0;
        
        receivedMsg.ID = 0x100 + (mockCounter % 3);
        receivedMsg.LEN = 4;
        receivedMsg.DATA[0] = 0xAA; // e.g., Axis ID
        receivedMsg.DATA[1] = 0x02; // e.g., Target Velocity Command
        receivedMsg.DATA[2] = 0x00;
        receivedMsg.DATA[3] = (BYTE)(mockCounter & 0xFF);
        
        // Dynamically rotate message type statuses for testing requirements
        if (mockCounter % 4 == 0) receivedMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        else if (mockCounter % 4 == 1) receivedMsg.MSGTYPE = PCAN_MESSAGE_STATUS;
        else if (mockCounter % 4 == 2) receivedMsg.MSGTYPE = PCAN_MESSAGE_ERRFRAME;
        else receivedMsg.MSGTYPE = PCAN_MESSAGE_ECHO;

        messageAvailable = true;
        mockCounter++;

        // Pass message via application callback if received successfully
        if (messageAvailable && m_Callback) {
            m_Callback(receivedMsg);
        }
    }
}