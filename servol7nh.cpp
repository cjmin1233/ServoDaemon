#include <chrono>
#include <iostream>
#include <thread>

#include <QDebug>

#include "cia402.h"
#include "servol7nh.h"

// settling constants
static constexpr int SETTLING_TIMEOUT      = 5000;
static constexpr int SETTLING_STABLE_COUNT = 50;

bool ServoL7NH::checkL7NH(int slaveId)
{
    static constexpr uint32 manufacturer = 0x00007595;
    static constexpr uint32 id           = 0x00010001;

    const auto& slave = ec_slave[slaveId];

    return slave.eep_man == manufacturer && slave.eep_id == id;
}

int ServoL7NH::setupPDO()
{
    int success = 1; // SOEM callbacks expect 1 on success

    // --- [STEP 1] RXPDO Mapping Content (0x1600) ---
    uint16_t rxpdoIndex = cia402::IDX_RXPDO_MAPPING_1;
    uint8_t  zero       = 0;

    // Set mapping count to 0 to clear existing mappings
    success &= (ec_SDOwrite(m_slaveId, rxpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t rxpdoEntries[] = {
        cia402::ENTRY_RX_CONTROL_WORD,
        cia402::ENTRY_RX_MODES_OF_OP,
        cia402::ENTRY_RX_TARGET_POSITION,
        cia402::ENTRY_RX_TARGET_VELOCITY,
        cia402::ENTRY_RX_TARGET_TORQUE,
        cia402::ENTRY_RX_DIGITAL_OUTPUTS,
    };
    uint8_t                  entryCount  = sizeof(rxpdoEntries) / sizeof(rxpdoEntries[0]);
    static constexpr uint8_t pdoCountMax = 10; // maximum 10 pdo entries available
    if (entryCount > pdoCountMax) {
        qCritical() << "[ServoL7NH::setupPDO] Too many RXPDO entries:" << static_cast<int>(entryCount);

        return 0;
    }

    for (uint8_t i = 0; i < entryCount; ++i) {
        // Write each mapping entry
        success &= (ec_SDOwrite(m_slaveId, rxpdoIndex, i + 1, FALSE, sizeof(rxpdoEntries[i]), &rxpdoEntries[i], EC_TIMEOUTRXM) > 0);
    }
    // Finalize the mapping count
    success &= (ec_SDOwrite(m_slaveId, rxpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 2] TXPDO Mapping Content (0x1A00) ---
    uint16_t txpdoIndex = cia402::IDX_TXPDO_MAPPING_1;

    // same as rxpdo: clear existing mappings first
    success &= (ec_SDOwrite(m_slaveId, txpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t txpdoEntries[] = {
        cia402::ENTRY_TX_STATUS_WORD,
        cia402::ENTRY_TX_MODES_OF_OP_DISP,
        cia402::ENTRY_TX_ACTUAL_POSITION,
        cia402::ENTRY_TX_ACTUAL_VELOCITY,
        cia402::ENTRY_TX_ACTUAL_TORQUE,
        cia402::ENTRY_TX_POSITION_WINDOW,
        cia402::ENTRY_TX_DIGITAL_INPUTS,
        cia402::ENTRY_TX_ERROR_CODE,
    };
    entryCount = sizeof(txpdoEntries) / sizeof(txpdoEntries[0]);
    if (entryCount > pdoCountMax) {
        qCritical() << "[ServoL7NH::setupPDO] Too many TXPDO entries:" << static_cast<int>(entryCount);

        return 0;
    }

    for (uint8_t i = 0; i < entryCount; ++i) {
        // Write each mapping entry
        success &= (ec_SDOwrite(m_slaveId, txpdoIndex, i + 1, FALSE, sizeof(txpdoEntries[i]), &txpdoEntries[i], EC_TIMEOUTRXM) > 0);
    }
    // Finalize the mapping count
    success &= (ec_SDOwrite(m_slaveId, txpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 3] Sync Manager 2 (RxPDO) & 3 (TxPDO) Assignment ---
    // RxPDO
    success    &= (ec_SDOwrite(m_slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(m_slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 1, FALSE, sizeof(rxpdoIndex), &rxpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(m_slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // TxPDO
    success    &= (ec_SDOwrite(m_slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(m_slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 1, FALSE, sizeof(txpdoIndex), &txpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(m_slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    return success;
}

int ServoL7NH::setupPos()
{
    int success = 1; // SOEM callbacks expect 1 on success

    // Set position objects
    uint32_t profileVel = m_encoderPPR;
    uint32_t profileAcc = m_encoderPPR * 2;
    uint32_t profileDec = m_encoderPPR * 2;
    uint32_t stopDec    = m_encoderPPR * 10;

    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_PROFILE_VELOCITY, 0, FALSE, sizeof(profileVel), &profileVel, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_PROFILE_ACCEL, 0, FALSE, sizeof(profileAcc), &profileAcc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_PROFILE_DECEL, 0, FALSE, sizeof(profileDec), &profileDec, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_STOP_DECEL, 0, FALSE, sizeof(stopDec), &stopDec, EC_TIMEOUTRXM) > 0);

    return success;
}

int ServoL7NH::setupHome()
{
    int success = 1; // SOEM callbacks expect 1 on success

    // Set homing objects
    int32_t  homeOffset   = m_encoderPPR / 200;
    int8_t   homingMethod = cia402::HM_NEG_LIMIT_SWITCH_AND_INDEX;
    uint32_t spdSwitch    = m_encoderPPR;
    uint32_t spdZero      = m_encoderPPR / 10;
    uint32_t homingAcc    = m_encoderPPR * 2;

    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_HOME_OFFSET, 0, FALSE, sizeof(homeOffset), &homeOffset, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_HOMING_METHOD, 0, FALSE, sizeof(homingMethod), &homingMethod, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_HOMING_SPEED, 1, FALSE, sizeof(spdSwitch), &spdSwitch, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_HOMING_SPEED, 2, FALSE, sizeof(spdZero), &spdZero, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_HOMING_ACCEL, 0, FALSE, sizeof(homingAcc), &homingAcc, EC_TIMEOUTRXM) > 0);
    return success;
}

int ServoL7NH::setupTorque()
{
    int success = 1; // SOEM callbacks expect 1 on success

    // Setup torque objects
    uint16_t torqueLimitFunc  = 2;
    uint16_t speedLimitFunc   = 0;
    uint16_t posTorqueLimit   = 3000;
    uint16_t negTorqueLimit   = 3000;
    uint16_t torqueSpeedLimit = 1000;
    uint32_t torqueSlope      = 1000;
    int16_t  torqueOffset     = 0;

    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_TORQUE_LIMIT_FUNCTION, 0, FALSE, sizeof(torqueLimitFunc), &torqueLimitFunc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_SPEED_LIMIT_FUNCTION, 0, FALSE, sizeof(speedLimitFunc), &speedLimitFunc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_POSITIVE_TORQUE_LIMIT, 0, FALSE, sizeof(posTorqueLimit), &posTorqueLimit, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_NEGATIVE_TORQUE_LIMIT, 0, FALSE, sizeof(negTorqueLimit), &negTorqueLimit, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_TORQUE_SPEED_LIMIT, 0, FALSE, sizeof(torqueSpeedLimit), &torqueSpeedLimit, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_TORQUE_SLOPE, 0, FALSE, sizeof(torqueSlope), &torqueSlope, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_TORQUE_OFFSET, 0, FALSE, sizeof(torqueOffset), &torqueOffset, EC_TIMEOUTRXM) > 0);

    return success;
}

#if 0
int ServoL7NH::setup(uint16 slaveId)
{
    // Note) Current servo drive rotation direction(0x2004) should be set to 1(cw is positive)
    // because of the NOT sensor position
    qInfo() << "[ServoL7NH::setupL7NH] Setup servo " << slaveId << " start";

    // Verify it's name starts with "L7NH"
    if (std::string(ec_slave[slaveId].name).find("L7NH") != 0) return 0;

    int success = 1; // SOEM callbacks expect 1 on success

    // Set PDO mappings
    success &= setupPDO(slaveId);

    // Set position objects
    success &= setupPosition(slaveId);

    // Set homing objects
    success &= setupHoming(slaveId);

    // Setup torque objects
    success &= setupTorque(slaveId);

    // etc...
    uint32_t posWindow      = s_encoderResolution / 200;
    int16_t  stopOption     = 2;
    int16_t  shutdownOption = 1;
    int16_t  haltOption     = 2;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, sizeof(posWindow), &posWindow, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_QUICK_STOP_OPTION, 0, FALSE, sizeof(stopOption), &stopOption, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_SHUTDOWN_OPTION, 0, FALSE, sizeof(shutdownOption), &shutdownOption, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_HALT_OPTION, 0, FALSE, sizeof(haltOption), &haltOption, EC_TIMEOUTRXM) > 0);

    qInfo() << "[ServoL7NH::setupL7NH] Result: " << (success ? "Success" : "Failed");

    return success;
}

int ServoL7NH::setupPDO(uint16 slaveId)
{
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
        cia402::ENTRY_RX_TARGET_TORQUE,
        cia402::ENTRY_RX_DIGITAL_OUTPUTS,
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
        cia402::ENTRY_TX_DIGITAL_INPUTS,
        cia402::ENTRY_TX_ERROR_CODE,
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

    return success;
}

int ServoL7NH::setupPosition(uint16 slaveId)
{
    int success = 1; // SOEM callbacks expect 1 on success

    // Set position objects
    uint32_t profileVel = s_encoderResolution;
    uint32_t profileAcc = s_encoderResolution * 2;
    uint32_t profileDec = s_encoderResolution * 2;
    uint32_t stopDec    = s_encoderResolution * 10;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_VELOCITY, 0, FALSE, sizeof(profileVel), &profileVel, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_ACCEL, 0, FALSE, sizeof(profileAcc), &profileAcc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_DECEL, 0, FALSE, sizeof(profileDec), &profileDec, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_STOP_DECEL, 0, FALSE, sizeof(stopDec), &stopDec, EC_TIMEOUTRXM) > 0);

    return success;
}

int ServoL7NH::setupHoming(uint16 slaveId)
{
    int success = 1; // SOEM callbacks expect 1 on success

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
    return success;
}

int ServoL7NH::setupTorque(uint16 slaveId)
{
    int success = 1; // SOEM callbacks expect 1 on success

    // Setup torque objects
    uint16_t torqueLimitFunc  = 2;
    uint16_t speedLimitFunc   = 0;
    uint16_t posTorqueLimit   = 3000;
    uint16_t negTorqueLimit   = 3000;
    uint16_t torqueSpeedLimit = 1000;
    uint32_t torqueSlope      = 1000;
    int16_t  torqueOffset     = 0;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_TORQUE_LIMIT_FUNCTION, 0, FALSE, sizeof(torqueLimitFunc), &torqueLimitFunc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_SPEED_LIMIT_FUNCTION, 0, FALSE, sizeof(speedLimitFunc), &speedLimitFunc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_POSITIVE_TORQUE_LIMIT, 0, FALSE, sizeof(posTorqueLimit), &posTorqueLimit, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_NEGATIVE_TORQUE_LIMIT, 0, FALSE, sizeof(negTorqueLimit), &negTorqueLimit, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_TORQUE_SPEED_LIMIT, 0, FALSE, sizeof(torqueSpeedLimit), &torqueSpeedLimit, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_TORQUE_SLOPE, 0, FALSE, sizeof(torqueSlope), &torqueSlope, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_TORQUE_OFFSET, 0, FALSE, sizeof(torqueOffset), &torqueOffset, EC_TIMEOUTRXM) > 0);

    return success;
}
#endif

ServoL7NH::ServoL7NH(uint16_t slaveId)
    : Slave(slaveId)
{
}

void ServoL7NH::setNone()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode             = 0;                              // Set to mode None
    rxpdo->target_position  = 0;                              // Set target position 0
    rxpdo->target_torque    = 0;                              // Clear target torque
    rxpdo->control_word    &= ~(cia402::CW_BIT_ABS_REL);      // Absolute move
    rxpdo->control_word    &= ~(cia402::CW_BIT_NEW_SETPOINT); // Clear homing start bit

    // rxpdo->control_word    &= ~(cia402::CW_BIT_HALT);         // Clear halt bit
    rxpdo->control_word |= cia402::CW_BIT_HALT; // Turn on halt bit to stop

    m_flagHomingStart = false;
    m_flagNewSetpoint = false;
    m_isSettling      = false;
}

void ServoL7NH::processData()
{
    auto* rxpdo = ptrRxPDO();

    const auto* txpdo      = ptrTxPDO();
    const auto& statusWord = txpdo->status_word;

    //     if (statusWord & cia402::SW_BIT_WARNING_OCCURED) {
    //         qWarning() << "[ServoL7NH::processData] Warning Detected!";
    //     }

    if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_OP_ENABLED) {
        const auto& currentMode = static_cast<cia402::Mode>(txpdo->mode_disp);

        // main operation
        switch (currentMode) {
        case cia402::Mode::None:
            break; // No operation mode selected
        case cia402::Mode::PP: {
            processPP(rxpdo, txpdo);

            break;
        }
        case cia402::Mode::PV:
            break; // Profile Velocity
        case cia402::Mode::PT: {
            processPT(rxpdo, txpdo);

            break;
        }
        case cia402::Mode::HM: {
            processHM(rxpdo, txpdo);

            break;
        }
        case cia402::Mode::CSP:
        case cia402::Mode::CST:
        case cia402::Mode::CSV:
            break;
        default:
            qInfo() << "[ServoL7NH::processData] invalid mode : " << static_cast<int8_t>(currentMode);
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

int ServoL7NH::setup()
{
    qInfo() << "[ServoL7NH::setup] Setup servo " << m_slaveId << " start";

    int success = 1; // SOEM callbacks expect 1 on success

    // Note) Current servo drive rotation direction(0x2004) should be set to 1(cw is positive)
    // because of the NOT sensor position

    // Read encoder ppr(pulse per revolution)
    int psize = sizeof(m_encoderPPR);
    ec_SDOread(m_slaveId, cia402::IDX_ENCODER_PPR, 0, FALSE, &psize, &m_encoderPPR, EC_TIMEOUTRXM);
    // // Read position window
    // ec_SDOread(m_slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, &psize, &m_posWindow, EC_TIMEOUTRXM);

    if (m_encoderPPR == 0) {
        qCritical() << "[ServoL7NH::setup] Encoder ppr not read";
        return 0;
    }
    // if (m_posWindow == 0) {
    //     qCritical() << "[ServoL7NH::setup] Position window not read";
    //     return 0;
    // }

    // Verify it's name starts with "L7NH"
    if (std::string(ec_slave[m_slaveId].name).find("L7NH") != 0) return 0;

    // Set PDO mappings
    success &= setupPDO();

    // Set position objects
    success &= setupPos();

    // Set homing objects
    success &= setupHome();

    // Setup torque objects
    success &= setupTorque();

    // etc...
    uint32_t posWindow  = getEncoderPPR() / 200;
    success            &= (ec_SDOwrite(m_slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, sizeof(posWindow), &posWindow, EC_TIMEOUTRXM) > 0);

    int16_t stopOption     = 2;
    int16_t shutdownOption = 1;
    int16_t haltOption     = 2;

    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_QUICK_STOP_OPTION, 0, FALSE, sizeof(stopOption), &stopOption, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_SHUTDOWN_OPTION, 0, FALSE, sizeof(shutdownOption), &shutdownOption, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(m_slaveId, cia402::IDX_HALT_OPTION, 0, FALSE, sizeof(haltOption), &haltOption, EC_TIMEOUTRXM) > 0);

    qInfo() << "[ServoL7NH::setupL7NH] Result: " << (success ? "Success" : "Failed");

    return success;
}

void ServoL7NH::start()
{
    // int psize = sizeof(m_posWindow);
    // // read position window setting
    // ec_SDOread(m_slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, &psize, &m_posWindow, EC_TIMEOUTRXM);

    // start command: homing mode
    setHome();
}

void ServoL7NH::stop()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->control_word = cia402::CW_SHUTDOWN; // 0x0006
}

void ServoL7NH::setTargetPosition(float ratio)
{
    // static constexpr int32_t maxPosition = s_encoderResolution * 4;
    const int32_t maxPosition = getEncoderPPR() * 4;

    int32_t pos = static_cast<int32_t>(ratio * (maxPosition));

    setTargetPosition(pos);
}

void ServoL7NH::setTargetPosition(int32_t pos) // TODO: uint32_t
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode             = static_cast<int8_t>(cia402::Mode::PP);
    rxpdo->target_position  = pos;
    rxpdo->target_torque    = 0;                              // Clear target torque
    rxpdo->control_word    &= ~(cia402::CW_BIT_HALT);         // Clear halt bit
    rxpdo->control_word    &= ~(cia402::CW_BIT_ABS_REL);      // Absolute move
    rxpdo->control_word    &= ~(cia402::CW_BIT_NEW_SETPOINT); // Clear new setpoint bit

    m_flagNewSetpoint = true;
    m_isSettling      = false;
}

void ServoL7NH::setHome()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode             = static_cast<int8_t>(cia402::Mode::HM); // Set to Homing Mode
    rxpdo->target_position  = 0;                                     // Set target position 0...just in case
    rxpdo->target_torque    = 0;                                     // Clear target torque
    rxpdo->control_word    &= ~(cia402::CW_BIT_HALT);                // Clear halt bit
    rxpdo->control_word    &= ~(cia402::CW_BIT_ABS_REL);             // Absolute move
    rxpdo->control_word    &= ~(cia402::CW_BIT_NEW_SETPOINT);        // Clear homing start bit

    m_flagHomingStart = true;
    m_isSettling      = false;
}

void ServoL7NH::setTorque(int16_t torque)
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode           = static_cast<int8_t>(cia402::Mode::PT);
    rxpdo->control_word  &= ~(cia402::CW_BIT_HALT); // Clear halt bit
    rxpdo->target_torque  = 1200;

    m_isSettling = false;
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
        qInfo() << "[ServoL7NH::stateCheck] ecat state not op...";
        // return;
    }

    if (rxpdo == nullptr || txpdo == nullptr) {
        qInfo() << "[ServoL7NH::stateCheck] pdo is nullptr...";
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
    if ((statusWord & cia402::SW_STATE_MASK1) == cia402::SW_STATE_FAULT) {
        qWarning() << "[ServoL7NH::stateCheck] Servo FAULT Detected!";

        // Set control word
        controlWord &= bit0F;                  // clear bit 4 to 15
        controlWord |= cia402::CW_FAULT_RESET; // bit 7: Fault reset(0 -> 1)

        // Update status
        m_Status.hasError  = true;
        m_Status.errorCode = txpdo->error_code;

        // Clear pdo values
        rxpdo->mode          = 0;
        rxpdo->target_torque = 0;

        m_stateCheckCounter = stateCheckCycleCounter;
        return;
    } else {
        // Set control word
        controlWord &= ~(cia402::CW_FAULT_RESET); // bit 7: Fault reset(1 -> 0)

        // Update status
        m_Status.hasError  = false;
        m_Status.errorCode = 0;
    }

    qInfo() << "[ServoL7NH::stateCheck] servo state transitions...";

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

            qInfo() << "[ServoL7NH::stateCheck] Already tried to enable op. Drop to shutdown";
        } else {
            // Enable operation
            controlWord = controlWord & bitF0 | cia402::CW_ENABLE_OP;
        }

        // Longer delay before next check
        m_stateCheckCounter = stateCheckCycleCounter * 2;
    }
}

void ServoL7NH::processPP(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Profile position mode
    static constexpr int8_t MODE_PP = static_cast<int8_t>(cia402::Mode::PP);
    // operated mode should be already set to PP
    if (rxpdo->mode != MODE_PP) {
        return;
    }

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

    if (isInPosition(rxpdo, txpdo) && !m_isSettling) {
        qInfo() << "[ServoL7NH::processPP] Target reached. Start settling check...";

        m_isSettling            = true;
        m_settlingTimeout       = SETTLING_TIMEOUT;
        m_settlingStableCounter = 0;

        return;
    }

    // Settling phase logic
    if (m_isSettling) {
        settling(rxpdo, txpdo);
    }
}

void ServoL7NH::processPT(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Profile torque mode
    static constexpr int8_t MODE_PT = static_cast<int8_t>(cia402::Mode::PT);
    // operated mode should be already set to PT
    if (rxpdo->mode != MODE_PT) {
        return;
    }

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    const bool isWarning = statusWord & cia402::SW_BIT_WARNING_OCCURED;
    const bool isLimit   = statusWord & cia402::SW_BIT_INTERNAL_LIMIT;

    // if (isWarning) {
    //     qWarning() << "[ServoL7NH::processPT] PT Mode Warning Detected!";
    // }
    if (isLimit) {
        qWarning() << "[ServoL7NH::processPT] Internal Limit Active: Torque might be capped by drive parameters.";
    }
}

void ServoL7NH::processHM(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Homing mode
    static constexpr int8_t MODE_HM = static_cast<int8_t>(cia402::Mode::HM);
    // operated mode should be already set to HM
    if (rxpdo->mode != MODE_HM) {
        return;
    }

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    const bool isHomingStart    = controlWord & cia402::CW_BIT_NEW_SETPOINT;
    const bool isHomingAttained = statusWord & cia402::SW_BIT_HOMING_ATTAINED;
    const bool isHomingError    = statusWord & cia402::SW_BIT_HOMING_ERROR;

    // Homing error handling
    if (isHomingError) {
        qInfo() << "[ServoL7NH::processHM] homing error occurred, try to restart homing...";

        // restart homing
        controlWord       &= ~(cia402::CW_BIT_NEW_SETPOINT);
        m_flagHomingStart  = true;

        return;
    }

    // Homing start request, but not yet started
    if (m_flagHomingStart && !isHomingStart) {
        // start homing
        controlWord       |= cia402::CW_BIT_NEW_SETPOINT;
        m_flagHomingStart  = false; // Reset homing flag

        return;
    }

    // Homing processing...
    if (isHomingStart) {
        // Homing Attained, enter settling phase
        if (isHomingAttained && !m_isSettling) {
            qInfo() << "[ServoL7NH::processHM] Homing attained. Start settling check...";

            m_isSettling            = true;
            m_settlingTimeout       = SETTLING_TIMEOUT;
            m_settlingStableCounter = 0;

            return;
        }

        // Settling phase logic
        if (m_isSettling) {
            settling(rxpdo, txpdo);
        }
    }
}

void ServoL7NH::settling(RxPDO* rxpdo, const TxPDO* txpdo)
{
    auto& controlWord = rxpdo->control_word;

    --m_settlingTimeout; // Decrement timeout

    // Update stable counter whether target is reached
    m_settlingStableCounter = isInPosition(rxpdo, txpdo) ? m_settlingStableCounter + 1 : 0;

    // Success: Remained stable within the window for enough time
    if (m_settlingStableCounter >= SETTLING_STABLE_COUNT) {
        qInfo() << "[ServoL7NH::settling] Settling Succeeded (Stable in window)";

        // Reset mode
        setNone();
        // rxpdo->mode = 0;

        // controlWord  &= ~(cia402::CW_BIT_NEW_SETPOINT);
        // m_isSettling  = false;

        // controlWord |= cia402::CW_BIT_HALT; // Turn on halt bit to stop
    }
    // Failure: Timeout occurred before becoming stable
    else if (m_settlingTimeout <= 0) {
        qInfo() << "[ServoL7NH::settling] Settling Failed (Timeout, not stable)";

        // Reset mode
        setNone();
        // // Reset mode
        // rxpdo->mode = 0;

        // controlWord  &= ~(cia402::CW_BIT_NEW_SETPOINT);
        // m_isSettling  = false;
    }
}

const bool ServoL7NH::isInPosition(RxPDO* rxpdo, const TxPDO* txpdo) const
{
    const uint32_t posWindow = txpdo->position_window;
    // const uint32_t posWindow = getEncoderPPR() / 200;

    const int32_t  posDiff = rxpdo->target_position - txpdo->actual_position;
    const uint32_t absDiff = posDiff < 0 ? -posDiff : posDiff;

    // Check if within the position window
    return absDiff <= posWindow;
}

// void ServoL7NH::setEncoderPPR(uint32_t ppr)
// {
//     if (m_encoderPPR != 0) {
//         qWarning() << "[] encoderPPR is already initialized and cannot be changed";
//         return;
//     }

//     m_encoderPPR = ppr;
// }
