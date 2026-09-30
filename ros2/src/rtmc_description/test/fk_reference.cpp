#include "cDriveCalculator.h"
#include "cBodyKinematics.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    cDriveCalculator drive;
    RTMCCollision::cBodyKinematics bodies;
    AxelPostion q{};
    while (std::cin >> q.A1 >> q.A2 >> q.A3 >> q.A4 >> q.A5) {
        q.A1 *= 180.0 / M_PI; q.A2 *= 180.0 / M_PI; q.A3 *= 180.0 / M_PI;
        q.A4 *= 180.0 / M_PI; q.A5 *= 180.0 / M_PI;
        const auto frames = bodies.CalculateFrames(q);
        const auto fk = drive.CalculateForwardKinematics(q);
        std::cout << std::setprecision(17) << fk.X / 100.0 << ' ' << fk.Y / 100.0 << ' ' << fk.Yaw * M_PI / 180.0;
        for (const auto& frame : frames) {
            std::cout << ' ' << frame.translation.x << ' ' << frame.translation.y << ' ' << frame.translation.z;
            for (int row=0; row<3; ++row) for (int col=0; col<3; ++col) std::cout << ' ' << frame.rotation.m[row][col];
        }
        std::cout << '\n';
    }
}
