#include <chrono>
#include <iostream>
#include <thread>

#include <QDebug>

#include "cia402.h"
#include "servol7nh.h"

bool ServoL7NH::checkL7NH(int slaveId)
{
    const auto& slave = ec_slave[slaveId];
    return slave.eep_man == 0x00007595
           && slave.eep_id == 0x00010001;
}

int ServoL7NH::setupL7NH(uint16 slaveId)
{
    std::cout << "[ServoL7NH::setupL7NH] Setup servo " << slaveId << " start" << std::endl;

    // Verify it's name starts with "L7NH"
    if (std::string(ec_slave[slaveId].name).find("L7NH") != 0) return 0;

    int success = 1; // SOEM callbacks expect 1 on success

    // --- [STEP 1] RXPDO Mapping Content (0x1600) ---
    uint16_t rxpdoIndex = cia402::IDX_RXPDO_MAPPING_1;
    uint8_t  zero       = 0;

    // Set mapping count to 0 to clear existing mappings
    success &= (ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t rxpdoEntries[] = {
        cia402::ENTRY_RX_CONTROL_WORD,
        cia402::ENTRY_RX_MODES_OF_OP,
        cia402::ENTRY_RX_TARGET_POSITION,
        cia402::ENTRY_RX_TARGET_VELOCITY,
        cia402::ENTRY_RX_DIGITAL_OUTPUTS
    };
    uint8_t entryCount = sizeof(rxpdoEntries) / sizeof(rxpdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        // Write each mapping entry
        success &= (ec_SDOwrite(slaveId, rxpdoIndex, i + 1, FALSE, sizeof(rxpdoEntries[i]), &rxpdoEntries[i], EC_TIMEOUTRXM) > 0);
    }
    // Finalize the mapping count
    success &= (ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 2] TXPDO Mapping Content (0x1A00) ---
    uint16_t txpdoIndex = cia402::IDX_TXPDO_MAPPING_1;

    // same as rxpdo: clear existing mappings first
    success &= (ec_SDOwrite(slaveId, txpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t txpdoEntries[] = {
        cia402::ENTRY_TX_STATUS_WORD,
        cia402::ENTRY_TX_MODES_OF_OP_DISP,
        cia402::ENTRY_TX_ACTUAL_POSITION,
        cia402::ENTRY_TX_ACTUAL_VELOCITY,
        cia402::ENTRY_TX_ACTUAL_TORQUE,
        cia402::ENTRY_TX_DIGITAL_INPUTS
    };
    entryCount = sizeof(txpdoEntries) / sizeof(txpdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        // Write each mapping entry
        success &= (ec_SDOwrite(slaveId, txpdoIndex, i + 1, FALSE, sizeof(txpdoEntries[i]), &txpdoEntries[i], EC_TIMEOUTRXM) > 0);
    }
    // Finalize the mapping count
    success &= (ec_SDOwrite(slaveId, txpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 3] Sync Manager 2 (RxPDO) & 3 (TxPDO) Assignment ---
    // RxPDO
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 1, FALSE, sizeof(rxpdoIndex), &rxpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // TxPDO
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 1, FALSE, sizeof(txpdoIndex), &txpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    uint32_t profileVel = s_encoderResolution;
    uint32_t profileAcc = s_encoderResolution * 2;
    uint32_t profileDec = s_encoderResolution * 2;
    uint32_t stopDec    = s_encoderResolution * 10;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_VELOCITY, 0, FALSE, sizeof(profileVel), &profileVel, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_ACCEL, 0, FALSE, sizeof(profileAcc), &profileAcc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_DECEL, 0, FALSE, sizeof(profileDec), &profileDec, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_STOP_DECEL, 0, FALSE, sizeof(stopDec), &stopDec, EC_TIMEOUTRXM) > 0);

    // Set homing objects
    int32_t  homeOffset   = s_encoderResolution / 200;
    int8_t   homingMethod = cia402::HM_NEG_LIMIT_SWITCH_AND_INDEX;
    uint32_t spdSwitch    = s_encoderResolution;
    uint32_t spdZero      = s_encoderResolution / 10;
    uint32_t homingAcc    = s_encoderResolution * 2;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOME_OFFSET, 0, FALSE, sizeof(homeOffset), &homeOffset, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_METHOD, 0, FALSE, sizeof(homingMethod), &homingMethod, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_SPEED, 1, FALSE, sizeof(spdSwitch), &spdSwitch, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_SPEED, 2, FALSE, sizeof(spdZero), &spdZero, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_ACCEL, 0, FALSE, sizeof(homingAcc), &homingAcc, EC_TIMEOUTRXM) > 0);

    // etc...
    uint32_t posWindow = s_encoderResolution / 200;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, sizeof(posWindow), &posWindow, EC_TIMEOUTRXM) > 0);

    std::cout << "[ServoL7NH::setupL7NH] Result: " << (success ? "Success" : "Failed") << std::endl;
    return success;
}

void ServoL7NH::processData()
{
    auto* rxpdo = ptrRxPDO();

    const auto* txpdo       = ptrTxPDO();
    const auto& statusWord  = txpdo->status_word;
    const auto& currentMode = static_cast<cia402::Mode>(txpdo->mode_disp);

    if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_OP_ENABLED) {
        // main operation
        switch (currentMode) {
        case cia402::Mode::PP: {
            processPP(rxpdo, txpdo);

            break;
        }
        case cia402::Mode::PV:
        case cia402::Mode::PT:
            break; // Profile velocity / torque not implemented
        case cia402::Mode::HM: {
            processHM(rxpdo, txpdo);

            break;
        }
        case cia402::Mode::CSP:
        case cia402::Mode::CST:
        case cia402::Mode::CSV:
            break;
        default:
            std::cout << "[ServoL7NH::processData] invalid mode : " << static_cast<int8_t>(currentMode) << std::endl;
            break;
        }

        // update status
        m_Status.position = txpdo->actual_position;
        m_Status.velocity = txpdo->actual_velocity;
    }
    // check state machine if not operational yet
    else {
        stateCheck(rxpdo, txpdo);
    }
}

void ServoL7NH::start()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode          = static_cast<int8_t>(cia402::Mode::HM); // Set to Homing Mode
    rxpdo->control_word &= ~(cia402::CW_BIT_ABS_REL);             // Absolute move

    int psize = sizeof(m_posWindow);
    // read position window setting
    ec_SDOread(m_slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, &psize, &m_posWindow, EC_TIMEOUTRXM);

    m_flagHomingStart = true;
}

void ServoL7NH::stop()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->control_word = cia402::CW_SHUTDOWN; // 0x0006
}

void ServoL7NH::setTargetPosition(float ratio)
{
    static constexpr int32_t maxPosition = s_encoderResolution * 4;

    int32_t pos = static_cast<int32_t>(ratio * (maxPosition));

    setTargetPosition(pos);
}

void ServoL7NH::setTargetPosition(int32_t pos)
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode             = static_cast<int8_t>(cia402::Mode::PP);
    rxpdo->target_position  = pos;
    rxpdo->control_word    &= ~(cia402::CW_BIT_ABS_REL);      // Absolute move
    rxpdo->control_word    &= ~(cia402::CW_BIT_NEW_SETPOINT); // Clear new setpoint bit

    m_flagNewSetpoint = true;
}

void ServoL7NH::setHome()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode          = static_cast<int8_t>(cia402::Mode::HM); // Set to Homing Mode
    rxpdo->control_word &= ~(cia402::CW_BIT_ABS_REL);             // Absolute move
    rxpdo->control_word &= ~(cia402::CW_BIT_HOMING_START);        // Clear homing start bit

    m_flagHomingStart = true;
}

const bool ServoL7NH::isRunning() const
{
    const TxPDO* txpdo = ptrTxPDO();

    if (txpdo == nullptr) return false;

    const auto& statusWord = txpdo->status_word;

    return (statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_OP_ENABLED;
}

void ServoL7NH::stateCheck(RxPDO* rxpdo, const TxPDO* txpdo)
{
    static constexpr int      stateCheckCycleCounter = 20;
    static constexpr uint16_t bitF0                  = 0xF0;
    static constexpr uint16_t bit0F                  = 0x0F;

    // Only operate if in OPERATIONAL state
    if (ec_slave[m_slaveId].state != EC_STATE_OPERATIONAL) {
        qDebug() << "[ServoL7NH::stateCheck] ecat state not op...";
        // return;
    }

    if (rxpdo == nullptr || txpdo == nullptr) {
        qDebug() << "[ServoL7NH::stateCheck] pdo is nullptr...";
        return;
    }

    // Cycle delay for state check
    if (m_stateCheckCounter > 0) {
        --m_stateCheckCounter;
        return;
    }

    uint16_t&       controlWord = rxpdo->control_word;
    const uint16_t& statusWord  = txpdo->status_word;

    // Fault Reset
    if (statusWord & cia402::SW_STATE_FAULT) {
        qDebug() << "[ServoL7NH::stateCheck] servo state fault";

        controlWord |= cia402::CW_FAULT_RESET;

        m_stateCheckCounter = stateCheckCycleCounter;
        return;
    }

    qDebug() << "[ServoL7NH::stateCheck] servo state transitions...";

    // State machine transitions
    // from Switch On Disabled to Ready to Switch On
    if ((statusWord & cia402::SW_STATE_MASK1) == cia402::SW_STATE_SWITCH_ON_DISABLED) {
        controlWord = controlWord & bitF0 | cia402::CW_SHUTDOWN;

        m_stateCheckCounter = stateCheckCycleCounter;
    }
    // from Ready to Switch On to Switched On
    else if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_READY_SWITCH_ON) {
        controlWord = controlWord & bitF0 | cia402::CW_SWITCH_ON;

        m_stateCheckCounter = stateCheckCycleCounter;
    }
    // from Switched On to Operation Enabled
    else if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_SWITCHED_ON) {
        if ((controlWord & bit0F) == cia402::CW_ENABLE_OP) {
            // Already tried to enable op. Drop to shutdown
            // controlWord = controlWord & bitF0 | cia402::CW_SHUTDOWN;
            controlWord = cia402::CW_SHUTDOWN; // clear other bits

            std::cout << "[ServoL7NH::stateCheck] Already tried to enable op. Drop to shutdown" << std::endl;
        } else {
            // Enable operation
            controlWord = controlWord & bitF0 | cia402::CW_ENABLE_OP;
        }

        // Longer delay before next check
        m_stateCheckCounter = stateCheckCycleCounter * 2;
    }

    // qDebug() << "[ServoL7NH::stateCheck] current counter :" << m_stateCheckCounter;
}

void ServoL7NH::processPP(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Profile position mode
    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    const bool isNewSetpointRequested = controlWord & cia402::CW_BIT_NEW_SETPOINT;
    const bool isSetpointAck          = statusWord & cia402::SW_BIT_SET_POINT_ACK;

    if (isNewSetpointRequested) {
        if (isSetpointAck) {
            // new setpoint requested and acknowledged
            controlWord &= ~(cia402::CW_BIT_NEW_SETPOINT);
        }
    } else if (m_flagNewSetpoint) {
        // request new setpoint
        controlWord |= cia402::CW_BIT_NEW_SETPOINT;
        // flag off
        m_flagNewSetpoint = false;
    }
}

void ServoL7NH::processHM(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // homing mode
    static constexpr int HOMING_SETTLING_TIMEOUT = 5000;
    static constexpr int HOMING_STABLE_COUNT     = 50;

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    const bool isHomingStart    = controlWord & cia402::CW_BIT_HOMING_START;
    const bool isHomingAttained = statusWord & cia402::SW_BIT_HOMING_ATTAINED;
    const bool isHomingError    = statusWord & cia402::SW_BIT_HOMING_ERROR;

    if (isHomingError) {
        std::cout << "[ServoL7NH::processData] homing error occurred!" << std::endl;

        controlWord        &= ~(cia402::CW_BIT_HOMING_START);
        m_flagHomingStart   = false;
        m_isHomingSettling  = false;
        return;
    }

    // 1. Homing start request
    if (!isHomingStart && m_flagHomingStart) {
        controlWord |= cia402::CW_BIT_HOMING_START;

        m_flagHomingStart  = false;
        m_isHomingSettling = false; // Reset settling state on new homing start
        return;
    }

    // 2. Homing processing
    if (isHomingStart) {
        // 2-1. Homing Attained, enter settling phase
        if (isHomingAttained && !m_isHomingSettling) {
            std::cout << "[ServoL7NH::processHM] Homing attained. Start settling check..." << std::endl;

            m_isHomingSettling      = true;
            m_homingSettlingCounter = HOMING_SETTLING_TIMEOUT;
            m_homingStableCounter   = 0;
        }

        // 2-2. Settling phase logic
        if (m_isHomingSettling) {
            --m_homingSettlingCounter; // Decrement timeout counter

            const int32_t  pos    = txpdo->actual_position;
            const uint32_t absPos = pos < 0 ? -pos : pos;

            // Check if within the position window
            if (absPos <= m_posWindow) {
                ++m_homingStableCounter; // Increment stable counter
            } else {
                m_homingStableCounter = 0; // Reset if it moves out
            }

            // Success: Remained stable within the window for enough time
            if (m_homingStableCounter >= HOMING_STABLE_COUNT) {
                std::cout << "[ServoL7NH::processHM] Homing Succeeded (Stable in window)" << std::endl;

                controlWord        &= ~(cia402::CW_BIT_HOMING_START);
                m_isHomingSettling  = false;

                // return to 0
                setTargetPosition(0);
            }
            // Failure: Timeout occurred before becoming stable
            else if (m_homingSettlingCounter <= 0) {
                std::cout << "[ServoL7NH::processHM] Homing Failed (Timeout, not stable)" << std::endl;

                controlWord        &= ~(cia402::CW_BIT_HOMING_START);
                m_isHomingSettling  = false;
            }
        }
    }
}
