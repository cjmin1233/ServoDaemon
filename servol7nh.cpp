// #include <chrono>
// #include <iostream>
// #include <thread>

#include <QDebug>

#include "servoconfig.h"
#include "servol7nh.h"

// settling constants
static constexpr int SETTLING_TIMEOUT      = 5000;
static constexpr int SETTLING_STABLE_COUNT = 50;

// --- utility functions ---
/**
 * @brief Simplified wrapper for ec_SDOwrite using C++ templates.
 * * Automatically determines data size using sizeof(T) and handles logging.
 * * @tparam T Data type (uint32_t, int16_t, etc.)
 * @param slaveId Index of the slave on the EtherCAT network.
 * @param index Object Dictionary Index (e.g., 0x6081).
 * @param subIndex Object Dictionary Sub-Index.
 * @param value The value to be written to the slave.
 * @param label Descriptive name for logging/debugging purposes.
 * @return True if successful, false if a communication error occurs.
 */
template <typename T>
bool sdoWrite(uint16 slaveId, uint16 index, uint8 subIndex, T value,
              const char* label = nullptr)
{
    T data = value;

    // Perform the actual SDO write operation
    int wres = ec_SDOwrite(slaveId, index, subIndex, FALSE, sizeof(T), &data,
                           EC_TIMEOUTRXM);

    if (wres > 0) {
        // Log success (uncomment for verbose debugging)
        /*
        qInfo() << "[SDO WRITE SUCCESS]" << (label ? label : "Unknown")
                << QString("Index: 0x%1:%2").arg(index, 4, 16,
        QChar('0')).arg(subIndex)
                << "Value:" << value;
        */
        return true;
    } else {
        // Log critical failure with detailed information
        qCritical()
            << "[SDO WRITE FAILED]" << (label ? label : "Unknown")
            << QString("Index: 0x%1:%2").arg(index, 4, 16, QChar('0')).arg(subIndex)
            << "Value:" << value;
        return false;
    }
}

float calcPulsePerMmf(int slaveId)
{
    const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

    // calculate pulse per mm
    float gearRatio = cfg.motorRevolutions / cfg.shaftRevolutions;

    return cfg.encoderPPR / (gearRatio * cfg.leadMm);
}

// uint32_t calcPulsePerMm(int slaveId)
// {
//     // const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

//     // // calculate pulse per mm
//     // uint32_t gearRatio = cfg.motorRevolutions / cfg.shaftRevolutions;

//     // return cfg.encoderPPR / (gearRatio * cfg.leadMm);

//     return (uint32_t)calcPulsePerMmf(slaveId);
// }

int32_t calcPosLimit(int slaveId)
{
    const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

    return cfg.strokeMm * calcPulsePerMmf(slaveId);
}
// -------------------------

bool ServoL7NH::checkL7NH(int slaveId)
{
    static constexpr uint32 manufacturer = 0x00007595;
    static constexpr uint32 id           = 0x00010001;

    const auto& slave = ec_slave[slaveId];

    return slave.eep_man == manufacturer && slave.eep_id == id;
}

int ServoL7NH::setup(uint16 slaveId)
{
    // Note) Current servo drive rotation direction(0x2004) should be set to 1(cw
    // is positive) because of the NOT sensor position
    qInfo() << "[ServoL7NH::setup] Setup servo " << slaveId << " start";

    // Verify it's name starts with "L7NH"
    if (std::string(ec_slave[slaveId].name).find("L7NH") != 0)
        return 0;

    bool ok = true;

    // Set PDO mappings
    ok &= setupPDO(slaveId);

    // Set position objects
    ok &= setupPosition(slaveId);

    // Set homing objects
    ok &= setupHoming(slaveId);

    // Setup torque objects
    ok &= setupTorque(slaveId);

    // etc...
    const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

    ok &= sdoWrite(slaveId, servoOD::IDX_POSITION_WINDOW, 0, cfg.positionWindow,
                   "Position Window");
    ok &= sdoWrite(slaveId, servoOD::IDX_QUICK_STOP_OPTION, 0,
                   cfg.quickStopOption, "Quick Stop Option");
    ok &= sdoWrite(slaveId, servoOD::IDX_SHUTDOWN_OPTION, 0, cfg.shutdownOption,
                   "Shutdown Option");
    ok &= sdoWrite(slaveId, servoOD::IDX_HALT_OPTION, 0, cfg.haltOption,
                   "Halt Option");

    int32_t posLimitMax  = calcPosLimit(slaveId) * 2; // set position limit (bigger enough than stroke)
    ok                  &= sdoWrite(slaveId, servoOD::IDX_POSITION_LIMIT, 2, posLimitMax,
                                    "Position Limit Max");

    // Mechanical Specs
    // *** effective after reboot ***
    ok &= sdoWrite(slaveId, servoOD::IDX_ROTATION_DIRECTION, 0,
                   cfg.rotationDirection, "Rotation Direction");
    ok &= sdoWrite(slaveId, servoOD::IDX_GEAR_RATIO, 1, cfg.motorRevolutions,
                   "Motor Revolutions");
    ok &= sdoWrite(slaveId, servoOD::IDX_GEAR_RATIO, 2, cfg.shaftRevolutions,
                   "Shaft Revolutions");

    qInfo() << "[ServoL7NH::setup] Result: " << (ok ? "Success" : "Failed");

    return ok ? 1 : 0;
}

bool ServoL7NH::setupPDO(uint16 slaveId)
{
    bool ok = true; // SOEM callbacks expect 1 on success

    // --- [STEP 1] RXPDO Mapping Content (0x1600) ---
    uint16_t rxpdoIndex = servoOD::IDX_RXPDO_MAPPING_1;
    uint8_t  zero       = 0;

    // Set mapping count to 0 to clear existing mappings
    ok &= sdoWrite(slaveId, rxpdoIndex, 0, zero, "RxPDO Map Count 0");

    uint32_t rxpdoEntries[] = {
        servoOD::ENTRY_RX_CONTROL_WORD,
        servoOD::ENTRY_RX_MODES_OF_OP,
        servoOD::ENTRY_RX_TARGET_POSITION,
        servoOD::ENTRY_RX_TARGET_VELOCITY,
        servoOD::ENTRY_RX_TARGET_TORQUE,
        servoOD::ENTRY_RX_DIGITAL_OUTPUTS,
    };
    uint8_t entryCount = sizeof(rxpdoEntries) / sizeof(rxpdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        // Write each mapping entry
        ok &= sdoWrite(slaveId, rxpdoIndex, i + 1, rxpdoEntries[i],
                       "RxPDO Map Entry");
    }
    // Finalize the mapping count
    ok &= sdoWrite(slaveId, rxpdoIndex, 0, entryCount, "RxPDO Map Count");

    // --- [STEP 2] TXPDO Mapping Content (0x1A00) ---
    uint16_t txpdoIndex = servoOD::IDX_TXPDO_MAPPING_1;

    // same as rxpdo: clear existing mappings first
    ok &= sdoWrite(slaveId, txpdoIndex, 0, zero, "TxPDO Map Count 0");

    uint32_t txpdoEntries[] = {
        servoOD::ENTRY_TX_STATUS_WORD,
        servoOD::ENTRY_TX_MODES_OF_OP_DISP,
        servoOD::ENTRY_TX_ACTUAL_POSITION,
        servoOD::ENTRY_TX_ACTUAL_VELOCITY,
        servoOD::ENTRY_TX_ACTUAL_TORQUE,
        servoOD::ENTRY_TX_DIGITAL_INPUTS,
        servoOD::ENTRY_TX_ERROR_CODE,
    };
    entryCount = sizeof(txpdoEntries) / sizeof(txpdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        // Write each mapping entry
        ok &= sdoWrite(slaveId, txpdoIndex, i + 1, txpdoEntries[i],
                       "TxPDO Map Entry");
    }
    // Finalize the mapping count
    ok &= sdoWrite(slaveId, txpdoIndex, 0, entryCount, "TxPDO Map Count");

    // --- [STEP 3] Sync Manager 2 (RxPDO) & 3 (TxPDO) Assignment ---
    // RxPDO
    ok         &= sdoWrite(slaveId, servoOD::IDX_SM2_RXPDO_ASSIGN, 0, zero,
                           "SM2 Assign Count 0");
    ok         &= sdoWrite(slaveId, servoOD::IDX_SM2_RXPDO_ASSIGN, 1, rxpdoIndex,
                           "SM2 Assign RxPDO");
    entryCount  = 1;
    ok         &= sdoWrite(slaveId, servoOD::IDX_SM2_RXPDO_ASSIGN, 0, entryCount,
                           "SM2 Assign Count");

    // TxPDO
    ok         &= sdoWrite(slaveId, servoOD::IDX_SM3_TXPDO_ASSIGN, 0, zero,
                           "SM3 Assign Count 0");
    ok         &= sdoWrite(slaveId, servoOD::IDX_SM3_TXPDO_ASSIGN, 1, txpdoIndex,
                           "SM3 Assign TxPDO");
    entryCount  = 1;
    ok         &= sdoWrite(slaveId, servoOD::IDX_SM3_TXPDO_ASSIGN, 0, entryCount,
                           "SM3 Assign Count");

    return ok;
}

bool ServoL7NH::setupPosition(uint16 slaveId)
{
    bool ok = true; // SOEM callbacks expect 1 on success

    const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

    // Set position objects
    ok &= sdoWrite(slaveId, servoOD::IDX_PROFILE_VELOCITY, 0, cfg.profileVelocity,
                   "Profile Velocity");
    ok &= sdoWrite(slaveId, servoOD::IDX_PROFILE_ACCEL, 0, cfg.profileAccel,
                   "Profile Accel");
    ok &= sdoWrite(slaveId, servoOD::IDX_PROFILE_DECEL, 0, cfg.profileDecel,
                   "Profile Decel");
    ok &= sdoWrite(slaveId, servoOD::IDX_STOP_DECEL, 0, cfg.stopDecel,
                   "Stop Decel");
    ok &= sdoWrite(slaveId, servoOD::IDX_POS_COMMAND_FILTER, 0,
                   cfg.posCommandFilter, "Position Command Filter");
    ok &= sdoWrite(slaveId, servoOD::IDX_POS_COMMAND_AVG_FILTER, 0,
                   cfg.posCommandAvgFilter, "Position Command Avg Filter");
    ok &= sdoWrite(slaveId, servoOD::IDX_POS_LIMIT_FUNCTION, 0, cfg.posLimitFunc,
                   "Position Limit Function");

    return ok;
}

bool ServoL7NH::setupHoming(uint16 slaveId)
{
    bool ok = true; // SOEM callbacks expect 1 on success

    const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

    // Set homing objects
    ok &= sdoWrite(slaveId, servoOD::IDX_HOME_OFFSET, 0, cfg.homeOffset,
                   "Home Offset");
    ok &= sdoWrite(slaveId, servoOD::IDX_HOMING_METHOD, 0, cfg.homingMethod,
                   "Homing Method");
    ok &= sdoWrite(slaveId, servoOD::IDX_HOMING_SPEED, 1, cfg.homingSpdSwitch,
                   "Homing Speed Switch");
    ok &= sdoWrite(slaveId, servoOD::IDX_HOMING_SPEED, 2, cfg.homingSpdZero,
                   "Homing Speed Zero");
    ok &= sdoWrite(slaveId, servoOD::IDX_HOMING_ACCEL, 0, cfg.homingAccel,
                   "Homing Accel");
    ok &= sdoWrite(slaveId, servoOD::IDX_MOVE_TO_ZERO_AFTER_HOMING, 0, cfg.moveToZeroAfterHoming,
                   "Move To Zero After Homing");

    return ok;
}

bool ServoL7NH::setupTorque(uint16 slaveId)
{
    bool ok = true; // SOEM callbacks expect 1 on success

    const auto& cfg = ServoConfig::SlaveConfigs[slaveId];

    // Setup torque objects
    ok &= sdoWrite(slaveId, servoOD::IDX_TORQUE_LIMIT_FUNCTION, 0,
                   cfg.torqueLimitFunc, "Torque Limit Func");
    ok &= sdoWrite(slaveId, servoOD::IDX_SPEED_LIMIT_FUNCTION, 0,
                   cfg.speedLimitFunc, "Speed Limit Func");
    ok &= sdoWrite(slaveId, servoOD::IDX_POSITIVE_TORQUE_LIMIT, 0,
                   cfg.posTorqueLimit, "Positive Torque Limit");
    ok &= sdoWrite(slaveId, servoOD::IDX_NEGATIVE_TORQUE_LIMIT, 0,
                   cfg.negTorqueLimit, "Negative Torque Limit");
    ok &= sdoWrite(slaveId, servoOD::IDX_TORQUE_SPEED_LIMIT, 0,
                   cfg.torqueSpeedLimit, "Torque Speed Limit");
    ok &= sdoWrite(slaveId, servoOD::IDX_TORQUE_SLOPE, 0, cfg.torqueSlope,
                   "Torque Slope");
    ok &= sdoWrite(slaveId, servoOD::IDX_TORQUE_OFFSET, 0, cfg.torqueOffset,
                   "Torque Offset");

    return ok;
}

void ServoL7NH::processData()
{
    auto* rxpdo = ptrRxPDO();

    const auto* txpdo      = ptrTxPDO();
    const auto& statusWord = txpdo->status_word;

    if (rxpdo == nullptr || txpdo == nullptr)
        return;

    bool isServoEnabled = (statusWord & servoOD::SW_STATE_MASK2) == servoOD::SW_STATE_OP_ENABLED;

    if (isServoEnabled) {
        const auto& currentMode = static_cast<servoOD::Mode>(txpdo->mode_disp);

        // main operation
        switch (currentMode) {
        case servoOD::Mode::None:
            break; // No operation mode selected
        case servoOD::Mode::PP: {
            processPP(rxpdo, txpdo);

            break;
        }
        case servoOD::Mode::PV:
            break; // Profile Velocity
        case servoOD::Mode::PT: {
            // processPT(rxpdo, txpdo);

            break;
        }
        case servoOD::Mode::HM: {
            processHM(rxpdo, txpdo);

            break;
        }
        case servoOD::Mode::CSP:
        case servoOD::Mode::CST:
        case servoOD::Mode::CSV:
            break;
        default:
            qInfo() << "[ServoL7NH::processData] invalid mode : "
                    << static_cast<int8_t>(currentMode);
            break;
        }

        // update status
        std::lock_guard<std::mutex> lock(m_statusMutex);
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
    const auto& cfg = ServoConfig::SlaveConfigs[m_slaveId];

    // m_posWindow = cfg.positionWindow;

    // calculate pulse per mm, limit
    m_pulsePerMmf = calcPulsePerMmf(m_slaveId);
    m_posLimit    = calcPosLimit(m_slaveId);
    m_strokeMm    = cfg.strokeMm;

    // // start command: homing mode
    // setHome();
}

ServoStatus ServoL7NH::getStatus() const
{
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_Status;
}

void ServoL7NH::stop()
{
    RxPDO* rxpdo = ptrRxPDO();
    if (rxpdo == nullptr)
        return;

    // CiA402 표준 방식: Control Word의 Halt 비트(Bit 8)를 1로 세트
    // 드라이브가 설정된 Profile Deceleration에 따라 내부적으로 안전하고 부드럽게
    // 정지합니다.
    rxpdo->control_word |= servoOD::CW_BIT_HALT;

    // 진행 중이던 New Setpoint 요청이 있다면 취소
    rxpdo->control_word &= ~(servoOD::CW_BIT_NEW_SETPOINT);

    rxpdo->target_torque = 0; // Clear target torque
    m_targetTorque       = 0;

    // 제어 상태 플래그 초기화
    m_flagNewSetpoint = false;
    m_flagHomingStart = false;
    // m_isSettling      = false;

    qInfo() << "[ServoL7NH::stop] Halt bit set for slave" << m_slaveId;
}

void ServoL7NH::setTargetPosition(float ratio)
{
    int32_t pos = m_posLimit * ratio;

    setTargetPosition(pos);
}

void ServoL7NH::setTargetPosition(int32_t pos)
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr)
        return;

    int32_t target_position = (int32_t)(pos * m_pulsePerMmf); // Calculate target position

    if (target_position < 0 || target_position > m_posLimit) {
        // target position out of bounds
        return;
    }

    rxpdo->mode            = static_cast<int8_t>(servoOD::Mode::PP);
    rxpdo->target_position = target_position;

    // rxpdo->target_torque = 0; // Clear target torque
    // m_targetTorque       = 0;

    rxpdo->control_word &= ~(servoOD::CW_BIT_HALT);         // Clear halt bit
    rxpdo->control_word &= ~(servoOD::CW_BIT_ABS_REL);      // Absolute move
    rxpdo->control_word &= ~(servoOD::CW_BIT_NEW_SETPOINT); // Clear new setpoint bit

    m_flagNewSetpoint = true;
    // m_lastTargetReached = false;
    // m_isSettling      = false;
}

void ServoL7NH::setHome()
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr)
        return;

    rxpdo->mode            = static_cast<int8_t>(servoOD::Mode::HM); // Set to Homing Mode
    rxpdo->target_position = 0;                                      // Set target position 0...just in case

    rxpdo->target_torque = 0; // Clear target torque
    m_targetTorque       = 0;

    rxpdo->control_word &= ~(servoOD::CW_BIT_HALT);         // Clear halt bit
    rxpdo->control_word &= ~(servoOD::CW_BIT_ABS_REL);      // Absolute move
    rxpdo->control_word &= ~(servoOD::CW_BIT_NEW_SETPOINT); // Clear homing start bit

    m_flagHomingStart = true;
    m_lastHMState     = servoOD::HomingState::NotStarted;
}

/*
void ServoL7NH::setTorque(int16_t torque)
{
    RxPDO* rxpdo = ptrRxPDO();

    if (rxpdo == nullptr) return;

    rxpdo->mode          = static_cast<int8_t>(servoOD::Mode::PT);
    rxpdo->control_word |= (servoOD::CW_BIT_HALT); // Clear halt bit
                                                   // rxpdo->target_torque  =
torque; m_targetTorque = torque;                       // Store target torque to
be applied in processPT

    // m_isSettling = false;
}
*/

void ServoL7NH::processCommand(const Command& cmd)
{
    const auto* txpdo = ptrTxPDO();

    if (txpdo == nullptr)
        return;

    const auto& currentMode = static_cast<servoOD::Mode>(txpdo->mode_disp);
    const auto& statusWord  = txpdo->status_word;

    // Stop command is always allowed even during homing
    if (cmd.cmdType == CommandType::StopServo) {
        stop();
        return;
    }

    // During homing, other commands are ignored (unless homing is completed)
    if (currentMode == servoOD::Mode::HM && m_lastHMState != servoOD::HomingState::Completed) {
        return;
    }

    switch (cmd.cmdType) {
    case CommandType::MovePosition:
        // 절대치가 유효하지 않으면 PP 모드 구동을 거부하고 Homing 요구
        if (!(statusWord & servoOD::SW_BIT_ABS_VALID)) {
            qWarning() << "[ServoL7NH::processCommand] Slave" << m_slaveId
                       << ": Absolute position invalid! Homing required.";
            return;
        }
        setTargetPosition(cmd.value);
        break;
    case CommandType::SetHome:
        // // 절대치가 이미 유효하다면 Homing 건너뛰고 즉시 PP 모드로 전환
        // if (statusWord & servoOD::SW_BIT_ABS_VALID) {
        //     qInfo() << "[ServoL7NH::processCommand] Slave" << m_slaveId
        //             << ": Absolute position already valid. Skipping homing.";
        //     setTargetPosition(0);
        //     return;
        // }
        setHome();
        break;
    case CommandType::SetTorque:
        // setTorque(cmd.value);
        break;
    // case CommandType::StopServo:
    //     stop();
    //     break;
    default:
        break;
    }
}

const bool ServoL7NH::isRunning() const
{
    const TxPDO* txpdo = ptrTxPDO();

    if (txpdo == nullptr)
        return false;

    const auto& statusWord = txpdo->status_word;

    return (statusWord & servoOD::SW_STATE_MASK2) == servoOD::SW_STATE_OP_ENABLED;
}

void ServoL7NH::stateCheck(RxPDO* rxpdo, const TxPDO* txpdo)
{
    static constexpr int stateCheckCycleCounter = 20;

    // Cycle delay for state check
    if (m_stateCheckCounter > 0) {
        --m_stateCheckCounter;
        return;
    }

    // Only operate if in OPERATIONAL state
    uint16_t currentState = ec_slave[m_slaveId].state;
    if (currentState != EC_STATE_OPERATIONAL) {
        if (m_lastEcatState != currentState) {
            qInfo() << "[ServoL7NH::stateCheck] ecat state NOT OP! Slave:"
                    << m_slaveId
                    << "State:" << QString("0x%1").arg(currentState, 0, 16)
                    << "ALStatus:"
                    << QString("0x%1").arg(ec_slave[m_slaveId].ALstatuscode, 4, 16,
                                           QChar('0'));
            m_lastEcatState = currentState;
        }
        return;
    }
    m_lastEcatState = currentState;

    if (rxpdo == nullptr || txpdo == nullptr) {
        if (m_lastPdoValid) {
            qInfo() << "[ServoL7NH::stateCheck] pdo is nullptr...";
            m_lastPdoValid = false;
        }
        return;
    }
    m_lastPdoValid = true;

    uint16_t&       controlWord = rxpdo->control_word;
    const uint16_t& statusWord  = txpdo->status_word;

    // Fault Reset
    if ((statusWord & servoOD::SW_STATE_MASK1) == servoOD::SW_STATE_FAULT) {
        qWarning() << "[ServoL7NH::stateCheck] Servo FAULT Detected!";

        // Set control word
        controlWord &= servoOD::CW_MASK_STATE_CONTROL; // clear state control bits (0-3)
        controlWord |= servoOD::CW_FAULT_RESET;        // bit 7: Fault reset(0 -> 1)

        // Update status
        {
            std::lock_guard<std::mutex> lock(m_statusMutex);
            m_Status.hasError  = true;
            m_Status.errorCode = txpdo->error_code;
        }

        // Clear pdo values
        rxpdo->mode          = 0;
        rxpdo->target_torque = 0;

        m_stateCheckCounter = stateCheckCycleCounter;
        return;
    } else {
        // Set control word
        controlWord &= ~(servoOD::CW_FAULT_RESET); // bit 7: Fault reset(1 -> 0)

        // Update status
        {
            std::lock_guard<std::mutex> lock(m_statusMutex);
            m_Status.hasError  = false;
            m_Status.errorCode = 0;
        }
    }

    // Only log if status word changed significantly (not just moving bits)
    uint16_t currentStatus = statusWord & servoOD::SW_STATE_MASK2;
    bool     stateChanged  = (currentStatus != m_lastStatusWord);
    m_lastStatusWord       = currentStatus;

    // State machine transitions
    // from Switch On Disabled to Ready to Switch On
    if ((statusWord & servoOD::SW_STATE_MASK1) == servoOD::SW_STATE_SWITCH_ON_DISABLED) {
        if (stateChanged)
            qInfo() << "[ServoL7NH::stateCheck] Transition: Switch On Disabled -> "
                       "Shutdown";
        controlWord = (controlWord & servoOD::CW_MASK_STATE_CONTROL) | servoOD::CW_SHUTDOWN;

        m_stateCheckCounter = stateCheckCycleCounter;
    }
    // from Ready to Switch On to Switched On
    else if ((statusWord & servoOD::SW_STATE_MASK2) == servoOD::SW_STATE_READY_SWITCH_ON) {
        if (stateChanged)
            qInfo() << "[ServoL7NH::stateCheck] Transition: Ready to Switch On -> "
                       "Switch On";
        controlWord = (controlWord & servoOD::CW_MASK_STATE_CONTROL) | servoOD::CW_SWITCH_ON;

        m_stateCheckCounter = stateCheckCycleCounter;
    }
    // from Switched On to Operation Enabled
    else if ((statusWord & servoOD::SW_STATE_MASK2) == servoOD::SW_STATE_SWITCHED_ON) {
        if ((controlWord & servoOD::CW_MASK_COMMAND_BITS) == servoOD::CW_ENABLE_OP) {
            // Already tried to enable op. Drop to shutdown
            // controlWord = controlWord & bitF0 | cia402::CW_SHUTDOWN;
            controlWord = servoOD::CW_SHUTDOWN; // clear other bits

            if (stateChanged)
                qInfo() << "[ServoL7NH::stateCheck] Already tried to enable op. Drop "
                           "to shutdown";
        } else {
            // Enable operation
            if (stateChanged)
                qInfo() << "[ServoL7NH::stateCheck] Transition: Switched On -> Enable "
                           "Operation";
            controlWord = (controlWord & servoOD::CW_MASK_STATE_CONTROL) | servoOD::CW_ENABLE_OP;
        }

        // Longer delay before next check
        m_stateCheckCounter = stateCheckCycleCounter * 2;
    }
}

void ServoL7NH::processPP(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Profile position mode
    static constexpr int8_t MODE_PP = static_cast<int8_t>(servoOD::Mode::PP);
    // operated mode should be already set to PP
    if (rxpdo->mode != MODE_PP) {
        return;
    }

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    const bool isNewSetpointRequested = controlWord & servoOD::CW_BIT_NEW_SETPOINT;
    const bool isSetpointAck          = statusWord & servoOD::SW_BIT_SET_POINT_ACK;

    if (isNewSetpointRequested) {
        if (isSetpointAck) {
            // new setpoint requested and acknowledged
            controlWord &= ~(servoOD::CW_BIT_NEW_SETPOINT);
            // m_lastTargetReached  = false;
        }

        m_lastTargetReached = false;
    } else if (m_flagNewSetpoint) {
        // request new setpoint
        controlWord |= servoOD::CW_BIT_NEW_SETPOINT;
        // flag off
        m_flagNewSetpoint = false;

        m_lastTargetReached = false;
    }

    std::lock_guard<std::mutex> lock(m_statusMutex);

    bool hasTargetReached = statusWord & servoOD::SW_BIT_TARGET_REACHED;

    bool isHandshakeInProgress = isNewSetpointRequested || isSetpointAck;
    bool validArrival          = hasTargetReached && !isHandshakeInProgress;

    if (!m_lastTargetReached && validArrival) {
        emit arrived(m_slaveId);

        m_lastTargetReached = true;
    }
}

/*
void ServoL7NH::processPT(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Profile torque mode
    static constexpr int8_t MODE_PT = static_cast<int8_t>(servoOD::Mode::PT);
    // operated mode should be already set to PT
    if (rxpdo->mode != MODE_PT) {
        rxpdo->target_torque = -100;

        return;
    }

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    static constexpr int16_t overloadWarning = 500; // 50%

    const int16_t overloadRatio = getOverloadRatio();
    // If overload warning is active, set torque to 0 to prevent damage.
Otherwise, use the target torque. int16_t safeTorque = overloadRatio >
overloadWarning ? 0 : m_targetTorque;

    rxpdo->target_torque = safeTorque;
}
*/

void ServoL7NH::processHM(RxPDO* rxpdo, const TxPDO* txpdo)
{
    // Homing mode
    static constexpr int8_t MODE_HM = static_cast<int8_t>(servoOD::Mode::HM);
    // operated mode should be already set to HM
    if (rxpdo->mode != MODE_HM) {
        return;
    }

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;

    // --- [1] Homing State Detection ---
    // Combine Bit 13 (Error), 12 (Attained), 10 (Target Reached)
    bool bit10 = (statusWord & servoOD::SW_BIT_TARGET_REACHED) != 0;
    bool bit12 = (statusWord & servoOD::SW_BIT_HOMING_ATTAINED) != 0;
    bool bit13 = (statusWord & servoOD::SW_BIT_HOMING_ERROR) != 0;

    // Simplified bit combination to state mapping
    servoOD::HomingState currentState;
    if (!bit13) {
        if (!bit12) {
            currentState = bit10 ? servoOD::HomingState::Interrupted
                                 : servoOD::HomingState::InProgress;
        } else {
            currentState = bit10 ? servoOD::HomingState::Completed
                                 : servoOD::HomingState::AttainedNotReached;
        }
    } else {
        currentState = bit10 ? servoOD::HomingState::ErrorStopped
                             : servoOD::HomingState::ErrorMoving;
    }

    // Log state transitions
    if (m_lastHMState != currentState) {
        switch (currentState) {
        case servoOD::HomingState::InProgress:
            qInfo() << "[ServoL7NH::processHM] Slave" << m_slaveId
                    << ": Homing procedure is in progress";
            break;
        case servoOD::HomingState::Interrupted:
            qWarning() << "[ServoL7NH::processHM] Slave" << m_slaveId
                       << ": Homing procedure is interrupted or not started";
            break;
        case servoOD::HomingState::AttainedNotReached:
            qInfo() << "[ServoL7NH::processHM] Slave" << m_slaveId
                    << ": Homing attained, moving to home offset...";
            break;
        case servoOD::HomingState::Completed:
            qInfo() << "[ServoL7NH::processHM] Slave" << m_slaveId
                    << ": Homing procedure completed successfully";
            break;
        case servoOD::HomingState::ErrorMoving:
            qCritical() << "[ServoL7NH::processHM] Slave" << m_slaveId
                        << ": Homing error occurred (moving)";
            break;
        case servoOD::HomingState::ErrorStopped:
            qCritical() << "[ServoL7NH::processHM] Slave" << m_slaveId
                        << ": Homing error occurred (stopped)";
            break;
        }
        m_lastHMState = currentState;
    }

    // --- [2] State-based Logic ---
    switch (currentState) {
    case servoOD::HomingState::InProgress:
    case servoOD::HomingState::AttainedNotReached:
        // Just wait
        break;

    case servoOD::HomingState::Completed:
        // Homing finished successfully
        if (controlWord & servoOD::CW_BIT_NEW_SETPOINT) {
            qInfo() << "[ServoL7NH::processHM] Slave" << m_slaveId
                    << ": Homing procedure completed successfully. Transitioning to "
                       "PP mode.";
            controlWord &= ~(servoOD::CW_BIT_NEW_SETPOINT);

            // Automatically switch to PP mode and set target to 0
            setTargetPosition(0);
        }
        break;

    case servoOD::HomingState::Interrupted:
        // Check if we need to start or restart
        if (m_flagHomingStart) {
            qInfo() << "[ServoL7NH::processHM] Slave" << m_slaveId
                    << ": Starting homing operation";
            controlWord       |= servoOD::CW_BIT_NEW_SETPOINT;
            m_flagHomingStart  = false;
        }
        break;

    case servoOD::HomingState::ErrorMoving:
    case servoOD::HomingState::ErrorStopped:
        // Handle error: Reset start bit and flag for retry
        if (controlWord & servoOD::CW_BIT_NEW_SETPOINT) {
            controlWord       &= ~(servoOD::CW_BIT_NEW_SETPOINT);
            m_flagHomingStart  = true; // Set flag to allow retry after error reset
        }
        break;
    }
}

// // Homing processing...
// if (isHomingStart) {
//     // Homing Attained, enter settling phase
//     if (isHomingAttained && !m_isSettling) {
//         qInfo() << "[ServoL7NH::processHM] Homing attained. Start settling
//         check...";

//         m_isSettling            = true;
//         m_settlingTimeout       = SETTLING_TIMEOUT;
//         m_settlingStableCounter = 0;

//         return;
//     }

//     // Settling phase logic
//     if (m_isSettling) {
//         settling(rxpdo, txpdo);
//     }
// }

/*
void ServoL7NH::settling(RxPDO* rxpdo, const TxPDO* txpdo)
{
    auto& controlWord = rxpdo->control_word;

    --m_settlingTimeout; // Decrement timeout

    // Update stable counter whether target is reached
    m_settlingStableCounter = isInPosition(rxpdo, txpdo) ?
m_settlingStableCounter + 1 : 0;

    // Success: Remained stable within the window for enough time
    if (m_settlingStableCounter >= SETTLING_STABLE_COUNT) {
        qInfo() << "[ServoL7NH::settling] Settling Succeeded (Stable in
window)";

        // // Reset mode
        // rxpdo->mode = 0;

        controlWord  &= ~(servoOD::CW_BIT_NEW_SETPOINT);
        m_isSettling  = false;

        controlWord |= servoOD::CW_BIT_HALT; // Turn on halt bit to stop
    }
    // Failure: Timeout occurred before becoming stable
    else if (m_settlingTimeout <= 0) {
        qInfo() << "[ServoL7NH::settling] Settling Failed (Timeout, not
stable)";

        // // Reset mode
        // rxpdo->mode = 0;

        controlWord  &= ~(servoOD::CW_BIT_NEW_SETPOINT);
        m_isSettling  = false;
    }
}
*/
