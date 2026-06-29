#pragma once
#include "commonDrive.h"

class cDriveCalculator {
public:
    cDriveCalculator() = default;
    ~cDriveCalculator() = default;

    // Core transformation APIs
    drivePosition CalculateNextPosition(const drivePosition& currentPos, const joystickSignal& signal, double deltaTime = TIME_DELTA);
    void CalculateInverseKinematics(const drivePosition& targetPos, AxelPostion& axelPos);
};