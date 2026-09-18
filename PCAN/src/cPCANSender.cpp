#include "cPCANSender.h"

cPCANSender::cPCANSender(TPCANHandle channel) 
    : m_Channel(channel), m_IsInitialized(false) {}

cPCANSender::~cPCANSender() {
    Uninitialize();
}

bool cPCANSender::Initialize(DWORD baudRate) {
    // In production: return CAN_Initialize(m_Channel, baudRate) == PCAN_ERROR_OK;
    std::cout << "[cPCANSender] Initialized channel " << (int)m_Channel << " at baudrate " << baudRate << "\n";
    m_IsInitialized = true;
    return true;
}

bool cPCANSender::Uninitialize() {
    if (m_IsInitialized) {
        // In production: CAN_Uninitialize(m_Channel);
        std::cout << "[cPCANSender] Uninitialized channel " << (int)m_Channel << "\n";
        m_IsInitialized = false;
    }
    return true;
}

bool cPCANSender::SendMessage(DWORD id, TPCANMessageType msgType, BYTE len, const BYTE* data) {
    if (!m_IsInitialized) {
        std::cerr << "[cPCANSender] Error: Sender not initialized.\n";
        return false;
    }

    if (len > 8) {
        std::cerr << "[cPCANSender] Error: Data length exceeds 8 bytes.\n";
        return false;
    }

    TPCANMsg msg;
    msg.ID = id;
    msg.MSGTYPE = msgType;
    msg.LEN = len;
    
    // Clear packet structure and copy payload if it's not a remote transmission request (RTR)
    for (int i = 0; i < 8; ++i) msg.DATA[i] = 0;
    if (data && (msgType != PCAN_MESSAGE_RTR)) {
        for (int i = 0; i < len; ++i) {
            msg.DATA[i] = data[i];
        }
    }

    // In production: DWORD result = CAN_Write(m_Channel, &msg);
    // return (result == PCAN_ERROR_OK);
    
    return true;
}
