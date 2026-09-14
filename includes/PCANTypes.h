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
#define DRIVE_MSG           0x001
#define START_DRIVE_MSG     0x002
#define STOP_DRIVE_MSG      0x003
#define DRIVE_SIGNAL_MSG    0x004

// Revision-3 coherent drive feedback. 0x201-0x205 remain actuator targets;
// these IDs describe the simulated drive's accepted position feedback.
#define DRIVE_STATUS_MSG       0x180
#define DRIVE_EPOCH_MSG        0x181
#define DRIVE_POSE_A1_MSG      0x301
#define DRIVE_POSE_A2_MSG      0x302
#define DRIVE_POSE_A3_MSG      0x303
#define DRIVE_POSE_A4_MSG      0x304
#define DRIVE_POSE_A5_MSG      0x305
#define DRIVE_POSE_COMMIT_MSG  0x306
#define DRIVE_SPEED_MSG        0x307
#define RTMC_CAN_PROTOCOL_VERSION 3

// Requested Custom CAN Message Structure
typedef struct {
    DWORD            ID;        // CAN ID (11-bit standard or 29-bit extended)
    TPCANMessageType MSGTYPE;   // Message type flags
    BYTE             LEN;       // Data Length Code (DLC): 0 to 8 bytes
    BYTE             DATA[8];   // Data payload
} TPCANMsg;
