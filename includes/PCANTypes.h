#pragma once

#include <cstdint>

// Standard types mimic from PCANBasic.h
typedef uint32_t DWORD;
typedef uint8_t  BYTE;
typedef uint16_t TPCANHandle;

// TPCANMessageType flags
typedef uint8_t TPCANMessageType;
#define PCAN_MESSAGE_STANDARD  0x00U
#define PCAN_MESSAGE_RTR       0x01U
#define PCAN_MESSAGE_ERRFRAME  0x40U
#define PCAN_MESSAGE_ECHO      0x20U
#define PCAN_MESSAGE_STATUS    0x80U

// PCAN Channel Definition Mock (e.g., PCAN-USB Channel 1)
#define PCAN_USBBUS1           0x51U
#define PCAN_BAUD_500K         0x001C
#define PCAN_ERROR_OK          0x0000U

// Target configuration identifier token macro mapping
#define DRIVE_MSG 0x001

// Requested Custom CAN Message Structure
typedef struct {
    DWORD            ID;        // CAN ID (11-bit standard or 29-bit extended)
    TPCANMessageType MSGTYPE;   // Message type flags
    BYTE             LEN;       // Data Length Code (DLC): 0 to 8 bytes
    BYTE             DATA[8];   // Data payload
} TPCANMsg;