#ifndef SERVOCONFIG_H
#define SERVOCONFIG_H

#include <cstdint>

namespace ServoConfig {
struct ServoParams {
    // --- [1] Profile Position Mode (PP) ---
    uint32_t profileVelocity; // 0x6081
    uint32_t profileAccel;    // 0x6083
    uint32_t profileDecel;    // 0x6084
    uint32_t stopDecel;       // 0x6085

    // --- [2] Homing Mode (HM) ---
    int32_t  homeOffset;      // 0x607C
    int8_t   homingMethod;    // 0x6098
    uint32_t homingSpdSwitch; // 0x6099:01
    uint32_t homingSpdZero;   // 0x6099:02
    uint32_t homingAccel;     // 0x609A

    // --- [3] Profile Torque Mode (PT) ---
    uint16_t torqueLimitFunc;  // 0x2110
    uint16_t speedLimitFunc;   // 0x230D
    uint16_t posTorqueLimit;   // 0x60E0
    uint16_t negTorqueLimit;   // 0x60E1
    uint16_t torqueSpeedLimit; // 0x230E
    uint32_t torqueSlope;      // 0x6087
    int16_t  torqueOffset;     // 0x60B2

    // --- [4] etc ---
    uint32_t positionWindow;  // 0x6067
    int16_t  quickStopOption; // 0x605A
    int16_t  shutdownOption;  // 0x605B
    int16_t  haltOption;      // 0x605D
};

const ServoParams SlaveConfigs[] = {
    {}, // Index 0(Dummy)
    {
     // Slave 1
        262144, 524288, 524288, 2621440, // PP: Vel, Acc, Dec, StopDec
        0, 1, 262144, 26214, 524288,     // HM: Offset, Method, Spd1, Spd2, Acc
        2, 0, 3000, 3000, 100, 1000, 0,  // PT: Torque settings
        1000, 2, 1, 2,                   // Etc: Win, Stop, Shutdown, Halt
    },
};
}

#endif // SERVOCONFIG_H
