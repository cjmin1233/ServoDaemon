#include "servoconfig.h"

namespace ServoConfig {

// 각 슬레이브의 기본(컴파일 타임) 설정값
// ConfigLoader::load() 호출 시 JSON 값으로 덮어씌워진다
ServoParams SlaveConfigs[slaveCountMax + 1] = {
    {}, // Index 0 (Dummy)

    // ----------------------------------------------------------------
    // Servo 1
    // ----------------------------------------------------------------
    {
        // --- [1] Profile Position Mode (PP) ---
        .profileVelocity  = ppr[1],         // 0x6081
        .profileAccel     = ppr[1] * 2,     // 0x6083
        .profileDecel     = ppr[1] * 2,     // 0x6084
        .stopDecel        = ppr[1] * 10,    // 0x6085
        .posCommandFilter = 5000,           // 0x2109
        .posLimitFunc     = 3,              // 0x2400

        // --- [2] Homing Mode (HM) ---
        .homeOffset       = 0,              // 0x607C
        .homingMethod     = 1,              // 0x6098
        .homingSpdSwitch  = ppr[1],         // 0x6099:01
        .homingSpdZero    = ppr[1] / 10,    // 0x6099:02
        .homingAccel      = ppr[1] * 2,     // 0x609A

        // --- [3] Profile Torque Mode (PT) ---
        .torqueLimitFunc  = 2,              // 0x2110
        .speedLimitFunc   = 0,              // 0x230D
        .posTorqueLimit   = 3'000,          // 0x60E0
        .negTorqueLimit   = 3'000,          // 0x60E1
        .torqueSpeedLimit = 200,            // 0x230E
        .torqueSlope      = 1'000,          // 0x6087
        .torqueOffset     = 0,              // 0x60B2

        // --- [4] etc ---
        // posWindow = encoderPPR / (gearRatio * leadMm * 100)
        .positionWindow   = ppr[1] / 5000,  // 0x6067
        .quickStopOption  = 2,              // 0x605A
        .shutdownOption   = 1,              // 0x605B
        .haltOption       = 2,              // 0x605D

        // --- Mechanical Specs ---
        .encoderPPR        = ppr[1],        // 0x2002
        .rotationDirection = 1,             // 0x2004
        .motorRevolutions  = 10,            // 0x6091:01
        .shaftRevolutions  = 1,             // 0x6091:02
        .leadMm            = 5,             // Distance per motor revolution (mm)
        .strokeMm          = 200,           // Maximum travel range (mm)
    },

    // ----------------------------------------------------------------
    // Servo 2
    // ----------------------------------------------------------------
    {
        // --- [1] Profile Position Mode (PP) ---
        .profileVelocity  = ppr[2],         // 0x6081
        .profileAccel     = ppr[2] * 2,     // 0x6083
        .profileDecel     = ppr[2] * 2,     // 0x6084
        .stopDecel        = ppr[2] * 10,    // 0x6085
        .posCommandFilter = 5000,           // 0x2109
        .posLimitFunc     = 3,              // 0x2400

        // --- [2] Homing Mode (HM) ---
        .homeOffset       = 0,              // 0x607C
        .homingMethod     = 1,              // 0x6098
        .homingSpdSwitch  = ppr[2],         // 0x6099:01
        .homingSpdZero    = ppr[2] / 10,    // 0x6099:02
        .homingAccel      = ppr[2] * 2,     // 0x609A

        // --- [3] Profile Torque Mode (PT) ---
        .torqueLimitFunc  = 2,              // 0x2110
        .speedLimitFunc   = 0,              // 0x230D
        .posTorqueLimit   = 3'000,          // 0x60E0
        .negTorqueLimit   = 3'000,          // 0x60E1
        .torqueSpeedLimit = 200,            // 0x230E
        .torqueSlope      = 1'000,          // 0x6087
        .torqueOffset     = 0,             // 0x60B2

        // --- [4] etc ---
        // posWindow = encoderPPR / (gearRatio * leadMm * 100)
        .positionWindow   = ppr[2] / 10000, // 0x6067
        .quickStopOption  = 2,              // 0x605A
        .shutdownOption   = 1,              // 0x605B
        .haltOption       = 2,              // 0x605D

        // --- Mechanical Specs ---
        .encoderPPR        = ppr[2],        // 0x2002
        .rotationDirection = 0,             // 0x2004
        .motorRevolutions  = 10,            // 0x6091:01
        .shaftRevolutions  = 1,             // 0x6091:02
        .leadMm            = 10,            // Distance per motor revolution (mm)
        .strokeMm          = 300,           // Maximum travel range (mm)
    },
};

} // namespace ServoConfig
