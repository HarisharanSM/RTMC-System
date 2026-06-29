#include "../include/cCANDriveHandler.h"

joystickSignal cCANDriveHandler::ConvertToJoystickSignal(const TPCANMsg& msg) const {
    joystickSignal signal{0.0, 0.0, 0.0, 0.0};

    if (msg.LEN < 1) return signal;
    BYTE dataByte = msg.DATA[0];

    // Isolate bits safely using bitwise masks
    bool bit0 = (dataByte & (1 << 0)) != 0; // R-up
    bool bit1 = (dataByte & (1 << 1)) != 0; // R-down
    bool bit2 = (dataByte & (1 << 2)) != 0; // R-left
    bool bit3 = (dataByte & (1 << 3)) != 0; // R-right
    bool bit4 = (dataByte & (1 << 4)) != 0; // L-up
    bool bit5 = (dataByte & (1 << 5)) != 0; // L-down
    bool bit6 = (dataByte & (1 << 6)) != 0; // L-left
    bool bit7 = (dataByte & (1 << 7)) != 0; // L-right

    // Map x component (Bits 0 & 1)
    if (bit0 && !bit1)       signal.x = 1.0;
    else if (!bit0 && bit1)  signal.x = -1.0;

    // Map y component (Bits 2 & 3)
    if (bit2 && !bit3)       signal.y = 1.0;
    else if (!bit2 && bit3)  signal.y = -1.0;

    // Map CRAN component (Bits 4 & 5)
    if (bit4 && !bit5)       signal.CRAN = 1.0;
    else if (!bit4 && bit5)  signal.CRAN = -1.0;

    // Map LAO component (Bits 6 & 7)
    if (bit6 && !bit7)       signal.LAO = 1.0;
    else if (!bit6 && bit7)  signal.LAO = -1.0;

    return signal;
}