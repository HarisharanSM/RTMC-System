#include "../include/cDriveCalculator.h"
#include <cmath>
#include <iostream>

#define PI                  3.141592653589793
#define MAX_SPEED           60.0f                          //In degrees per second
#define MAX_DEG_PER_STEP    MAX_SPEED * TIME_DELTA/1000.0f //In degrees
#define ACC_RATE            120.0f                         //In degrees per second squared
#define LINK1_LEN           75.0f                          //In cm
#define LINK2_LEN           100.0f                         //In cm
#define MAX_X_PER_STEP      LINK1_LEN * std::cos(MAX_DEG_PER_STEP * PI / 180.0) + LINK2_LEN * std::cos((MAX_DEG_PER_STEP * PI / 180.0) *2)
#define MAX_Y_PER_STEP      LINK1_LEN * std::sin(MAX_DEG_PER_STEP * PI / 180.0) + LINK2_LEN * std::sin((MAX_DEG_PER_STEP * PI / 180.0) *2)
#define MAX_LAO             180.0f
#define MAX_CRAN            180.0f
#define MIN_LAO             -180.0f
#define MIN_CRAN            -180.0f
#define MAX_X               300.0f
#define MAX_Y               25.0f   
#define MIN_X               0.0f
#define MIN_Y               -25.0f
// Define where your robot base (0,0) is physically located in your world frame
#define BASE_X              50.0f  
#define BASE_Y              50.0f

drivePosition cDriveCalculator::CalculateNextPosition(const drivePosition& currentPos, const joystickSignal& signal, double deltaTime/*=TIME_DELTA*/) {
    drivePosition updatedPos = currentPos;
    AxelPostion currAxelPos;
    CalculateInverseKinematics(currentPos, currAxelPos);

    float fTramp = MAX_SPEED / ACC_RATE; // time spent ramping
    float fThetaRamp = 0.5 * MAX_SPEED * fTramp; // distance covered during ramping
    //Determine if you even hit max speed, with total distance required just to speed up and slow down
    float fThetaTotal = 2 * fThetaRamp;

    updatedPos.X += signal.x;
    updatedPos.Y += signal.y;
    updatedPos.LAO += signal.LAO;
    updatedPos.CRAN += signal.CRAN;

    AxelPostion axelPos;
    CalculateInverseKinematics(updatedPos, axelPos);

    double fDeltaA1 = std::abs(axelPos.A1 - currAxelPos.A1);
    double fDeltaA2 = std::abs(axelPos.A2 - currAxelPos.A2);

    //check feasability of the next position based on the max speed and acceleration constraints
    if(fThetaTotal < fDeltaA1 || fThetaTotal < fDeltaA2) {
        //If the axle clips early and forms a triangle, the actual peak velocity reached is
        float fVpeak = std::sqrt(ACC_RATE * fThetaTotal);
        float fAverageSpeed = fVpeak / 2.0;

        float fThetaActual = fAverageSpeed * deltaTime / 1000.0; // distance covered in this time step
        float fMax_X = LINK1_LEN * std::cos(fThetaActual * PI / 180.0) + LINK2_LEN * std::cos((fThetaActual * PI / 180.0) *2);
        float fMax_Y = LINK1_LEN * std::sin(fThetaActual * PI / 180.0) + LINK2_LEN * std::sin((fThetaActual * PI / 180.0) *2);
        updatedPos.X = currentPos.X + signal.x * fMax_X * deltaTime/1000;
        updatedPos.Y = currentPos.Y + signal.y * fMax_Y * deltaTime/1000;
    }

    if(updatedPos.X > MAX_X) updatedPos.X = static_cast<double>(MAX_X);
    else if(updatedPos.X < MIN_X) updatedPos.X = static_cast<double>(MIN_X);

    if(updatedPos.Y > MAX_Y) updatedPos.Y = static_cast<double>(MAX_Y);
    else if(updatedPos.Y < MIN_Y) updatedPos.Y = static_cast<double>(MIN_Y);

    if(updatedPos.LAO > MAX_LAO) updatedPos.LAO = static_cast<double>(MAX_LAO);
    else if(updatedPos.LAO < MIN_LAO) updatedPos.LAO = static_cast<double>(MIN_LAO);

    if(updatedPos.CRAN > MAX_CRAN) updatedPos.CRAN = static_cast<double>(MAX_CRAN);
    else if(updatedPos.CRAN < MIN_CRAN) updatedPos.CRAN = static_cast<double>(MIN_CRAN);

    std::cout<< "[cDriveCalculator] Calculated next position: X=" << updatedPos.X << ", Y=" << updatedPos.Y << "\n";
    std::cout<< "[cDriveCalculator] Calculated next position: LAO=" << updatedPos.LAO << ", CRAN=" << updatedPos.CRAN << "\n";

    return updatedPos;
}

void cDriveCalculator::CalculateInverseKinematics(const drivePosition& targetPos, AxelPostion& axelPos){
    double X = targetPos.X - BASE_X; // Adjust for base offset
    double Y = targetPos.Y - BASE_Y; // Adjust for base offset

    // Compute distances from the localized robot base (0,0)
    float distance = std::sqrt((X * X) + (Y * Y));
    float min_reach = std::abs(LINK2_LEN - LINK1_LEN); // 25.0
    float max_reach = LINK1_LEN + LINK2_LEN; 

    // Prevent division by zero if world target matches the robot base exactly
    if (distance < 0.001f) {
        X = min_reach; // Default to pushing straight out along relative X axis
        Y = 0.0f;
        distance = min_reach;
    }
    // Auto-scale relative target if it falls inside the inner dead-zone radius
    else if (distance < min_reach) {
        float scale = min_reach / distance;
        X *= scale; 
        Y *= scale;
        distance = min_reach;
    }
    // Auto-scale relative target if it is too far away
    else if (distance > max_reach) {
        float scale = max_reach / distance;
        X *= scale;
        Y *= scale;
        distance = max_reach;
    }
    
    float num = (std::pow(X, 2) + std::pow(Y, 2) - std::pow(LINK1_LEN, 2) - std::pow(LINK2_LEN, 2));
    float den = 2 * LINK1_LEN * LINK2_LEN;
    float temp = num / den;

    float angle2 = std::atan2(std::sqrt(1 - std::pow(temp, 2)), temp); // Elbow UP solution
    float angle1 = std::atan2(Y, X) - std::atan2(LINK2_LEN * std::sin(angle2), LINK1_LEN + LINK2_LEN * std::cos(angle2));
    axelPos.A1 = angle1 * 180.0 / PI;
    axelPos.A2 = angle2 * 180.0 / PI;

    axelPos.A3 = targetPos.LAO;
    axelPos.A4 = targetPos.CRAN;

    if(axelPos.A3 > MAX_LAO) axelPos.A3 = MAX_LAO;
    if(axelPos.A3 < MIN_LAO) axelPos.A3 = MIN_LAO;
    if(axelPos.A4 > MAX_CRAN) axelPos.A4 = MAX_CRAN;
    if(axelPos.A4 < MIN_CRAN) axelPos.A4 = MIN_CRAN;
}