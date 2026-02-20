#ifndef SERVOCONFIG_H
#define SERVOCONFIG_H

#include <cstdint>

namespace ServoConfig {
struct ServoParams {
    // --- [1] Profile Position Mode (PP) ---
    uint32_t profileVelocity;  // 0x6081
    uint32_t profileAccel;     // 0x6083
    uint32_t profileDecel;     // 0x6084
    uint32_t stopDecel;        // 0x6085
    uint16_t posCommandFilter; // 0x2109
    uint16_t posLimitFunc;     // 0x2400

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

    // --- Mechanical Specs ---
    uint32_t encoderPPR;        // 0x2002;
    uint16_t rotationDirection; // 0x2004;
    uint32_t motorRevolutions;  // 0x6091:01
    uint32_t shaftRevolutions;  // 0x6091:02 -> Gear ratio = motor revolutions / shaft revolutions
    uint32_t leadMm;            // Distance per motor revolution (mm)
    uint32_t strokeMm;          // Maximum travel range (mm)
};

static constexpr uint32_t ppr[] = { 0 /* Dummy */, 1 << 18, 1 << 19 }; // 0x2002: encoder pulse per revolution (LS mecapion: automatically configured)

const ServoParams SlaveConfigs[] = {
    {}, // Index 0(Dummy)
    {
     // Servo 1
        ppr[1], ppr[1] * 2, ppr[1] * 2, ppr[1] * 10, 5000, 3,                      // PP: Vel, Acc, Dec, StopDec, etc
        0, 1, ppr[1], ppr[1] / 10, ppr[1] * 2,                                     // HM: Offset, Method, Spd1, Spd2, Acc
        2, 0, 3'000, 3'000, 100, 1'000, 0,                                         // PT: Torque settings
        ppr[1] / 5000 /* posWindow = ppr / (gearRatio * leadMm * 100) */, 2, 1, 2, // etc
        ppr[1], 1, 10, 1, 5, 200,                                                  // Mechanical Specs
    },
    {
     // Servo 2
        ppr[2], ppr[2] * 2, ppr[2] * 2, ppr[2] * 10, 5000, 3, // PP: Vel, Acc, Dec, StopDec, etc
        0, 1, ppr[2], ppr[2] / 10, ppr[2] * 2,                // HM: Offset, Method, Spd1, Spd2, Acc
        2, 0, 3'000, 3'000, 100, 1'000, 0,                    // PT: Torque settings
        ppr[2] / 10000 /* posWindow = ppr / (gearRatio * leadMm * 100) */, 2, 1, 2,                              // etc
        ppr[2], 0, 10, 1, 10, 300,                            // Mechanical Specs
    },
};
}

#endif // SERVOCONFIG_H
