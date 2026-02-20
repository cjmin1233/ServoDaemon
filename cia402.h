#ifndef CIA402_H
#define CIA402_H

/**
 * CiA402 Standard Definitions for EtherCAT Servo Drives
 * Optimized for L7NH and similar Servo Drives
 * Organized in namespace and enum class for C++ safety.
 */
#include <cstdint>

namespace cia402 {

// --- [1] Control Word Commands (0x6040) ---
constexpr uint16_t CW_SHUTDOWN    = 0x0006; // Command: Ready to Switch On
constexpr uint16_t CW_SWITCH_ON   = 0x0007; // Command: Switched On
constexpr uint16_t CW_ENABLE_OP   = 0x000F; // Command: Operation Enabled (Servo ON)
constexpr uint16_t CW_FAULT_RESET = 0x0080; // Command: Fault Reset (Rising Edge)

// --- [2] Status Word Masks & States (0x6041) ---
constexpr uint16_t SW_STATE_MASK1 = 0x004F;
constexpr uint16_t SW_STATE_MASK2 = 0x006F;

constexpr uint16_t SW_STATE_NOT_READY          = 0x0000;
constexpr uint16_t SW_STATE_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t SW_STATE_READY_SWITCH_ON    = 0x0021;
constexpr uint16_t SW_STATE_SWITCHED_ON        = 0x0023;
constexpr uint16_t SW_STATE_OP_ENABLED         = 0x0027;
constexpr uint16_t SW_STATE_QUICK_STOP_ACTIVE  = 0x0007;
constexpr uint16_t SW_STATE_FAULT_REACTION     = 0x000F;
constexpr uint16_t SW_STATE_FAULT              = 0x0008;

// --- Control Word (0x6040) Bit Flags ---
constexpr uint16_t CW_BIT_NEW_SETPOINT = (1 << 4); // bit 4: Apply new setpoint (PP & HM mode)
// constexpr uint16_t CW_BIT_HOMING_START           = (1 << 4); // bit 4: Start homing operation (HM mode)
constexpr uint16_t CW_BIT_CHANGE_SET_IMMEDIATELY = (1 << 5); // bit 5: Apply setpoint immediately (PP mode)
constexpr uint16_t CW_BIT_ABS_REL                = (1 << 6); // bit 6: Use Absolute(0) or Relative(1) position
constexpr uint16_t CW_BIT_HALT                   = (1 << 8); // bit 8: Halt motion

// --- Status Word (0x6041) Bit Flags ---
constexpr uint16_t SW_BIT_VOLTAGE_ENABLED = (1 << 4);  // Drive voltage is enabled
constexpr uint16_t SW_BIT_WARNING_OCCURED = (1 << 7);  // A warning is active
constexpr uint16_t SW_BIT_TARGET_REACHED  = (1 << 10); // Target position has been reached
constexpr uint16_t SW_BIT_SET_POINT_ACK   = (1 << 12); // Setpoint has been acknowledged (PP mode)
constexpr uint16_t SW_BIT_INTERNAL_LIMIT  = (1 << 11); // Internal limit active

// --- Homing Mode (HM) Specific Bits ---
constexpr uint16_t SW_BIT_HOMING_ATTAINED = (1 << 12); // Homing procedure is completed
constexpr uint16_t SW_BIT_HOMING_ERROR    = (1 << 13); // An error occurred during homing

// --- [3] Modes of Operation (0x6060) ---
enum class Mode : int8_t {
    None = 0,
    PP   = 1, // Profile Position Mode
    PV   = 3, // Profile Velocity Mode
    PT   = 4, // Profile Torque Mode
    HM   = 6, // Homing Mode
    CSP  = 8, // Cyclic Synchronous Position Mode
    CSV  = 9, // Cyclic Synchronous Velocity Mode
    CST  = 10 // Cyclic Synchronous Torque Mode
};

// --- [4] Object Dictionary Index ---
constexpr uint16_t IDX_CONTROL_WORD       = 0x6040;
constexpr uint16_t IDX_STATUS_WORD        = 0x6041;
constexpr uint16_t IDX_OP_MODE            = 0x6060;
constexpr uint16_t IDX_OP_MODE_DISPLAY    = 0x6061;
constexpr uint16_t IDX_TARGET_POSITION    = 0x607A;
constexpr uint16_t IDX_TARGET_TORQUE      = 0x6071;
constexpr uint16_t IDX_ACTUAL_POSITION    = 0x6064;
constexpr uint16_t IDX_POSITION_WINDOW    = 0x6067;
constexpr uint16_t IDX_PROFILE_VELOCITY   = 0x6081;
constexpr uint16_t IDX_PROFILE_ACCEL      = 0x6083;
constexpr uint16_t IDX_PROFILE_DECEL      = 0x6084;
constexpr uint16_t IDX_STOP_DECEL         = 0x6085;
constexpr uint16_t IDX_DIGITAL_INPUTS     = 0x60FD;
constexpr uint16_t IDX_GEAR_RATIO         = 0x6091; // SubIndex 1: Motor revolutions, SubIndex 2: Shaft revolutions -> unsigned 32 bit, effective after reboot
constexpr uint16_t IDX_POSITION_LIMIT     = 0x607D; // SubIndex 1: Min limit, SubIndex 2: Max limit
constexpr uint16_t IDX_POS_COMMAND_FILTER = 0x2109; // unsigned 16 bit, 0 to 10'000, default 0, 0.1ms, rw
constexpr uint16_t IDX_POS_LIMIT_FUNCTION = 0x2400; // unsigned 16 bit, 0 to 3, default 0, rw
constexpr uint16_t IDX_ROTATION_DIRECTION = 0x2004; // unsinged 16 bit, 0 to 1, default 0, rw, effective after reboot

// Option codes
constexpr uint16_t IDX_QUICK_STOP_OPTION = 0x605A;
constexpr uint16_t IDX_SHUTDOWN_OPTION   = 0x605B;
constexpr uint16_t IDX_DISABLE_OPTION    = 0x605C;
constexpr uint16_t IDX_HALT_OPTION       = 0x605D;

// Homing
constexpr uint16_t IDX_HOME_OFFSET   = 0x607C;
constexpr uint16_t IDX_HOMING_METHOD = 0x6098;
constexpr uint16_t IDX_HOMING_SPEED  = 0x6099;
constexpr uint16_t IDX_HOMING_ACCEL  = 0x609A;

// Torque
constexpr uint16_t IDX_TORQUE_LIMIT_FUNCTION = 0x2110; // unsigned 16 bit
constexpr uint16_t IDX_SPEED_LIMIT_FUNCTION  = 0x230D; // unsigned 16 bit
constexpr uint16_t IDX_POSITIVE_TORQUE_LIMIT = 0x60E0; // unsigned 16 bit
constexpr uint16_t IDX_NEGATIVE_TORQUE_LIMIT = 0x60E1; // unsigned 16 bit
constexpr uint16_t IDX_TORQUE_SPEED_LIMIT    = 0x230E; // unsigned 16 bit
constexpr uint16_t IDX_TORQUE_SLOPE          = 0x6087; // unsigned 32 bit
constexpr uint16_t IDX_TORQUE_OFFSET         = 0x60B2; // signed 16 bit

// --- [5] RxPDO Mapping Objects (Master -> Slave) ---
// Defines the content of data sent from Master to Slave.
constexpr uint16_t IDX_RXPDO_MAPPING_1 = 0x1600;
constexpr uint16_t IDX_RXPDO_MAPPING_2 = 0x1601;
constexpr uint16_t IDX_RXPDO_MAPPING_3 = 0x1602;
constexpr uint16_t IDX_RXPDO_MAPPING_4 = 0x1603;

// --- [6] TxPDO Mapping Objects (Slave -> Master) ---
// Defines the content of data sent from Slave to Master.
constexpr uint16_t IDX_TXPDO_MAPPING_1 = 0x1A00;
constexpr uint16_t IDX_TXPDO_MAPPING_2 = 0x1A01;
constexpr uint16_t IDX_TXPDO_MAPPING_3 = 0x1A02;
constexpr uint16_t IDX_TXPDO_MAPPING_4 = 0x1A03;

// --- [7] Sync Manager PDO Assignment Objects ---
// Assigns the PDO mapping objects (e.g., 0x1600, 0x1A00) to Sync Manager channels.
constexpr uint16_t IDX_SM2_RXPDO_ASSIGN = 0x1C12; // SM2 for RxPDO (Outputs from Master)
constexpr uint16_t IDX_SM3_TXPDO_ASSIGN = 0x1C13; // SM3 for TxPDO (Inputs to Master)

// --- Note on PDO Mapping Sub-Indices ---
// Sub-Index 0: Number of mapped objects in this PDO (uint8_t)
// Sub-Index 1~n: Information of the mapped object (uint32_t: Index/Sub-Index/BitLength)

// --- [8] RxPDO Mapping Entries (Master -> Slave) ---
// Format: Index(16bit) + SubIndex(8bit) + BitLength(8bit)

// Control Word (0x6040:00, 16bit) -> 0x60400010
constexpr uint32_t ENTRY_RX_CONTROL_WORD = 0x60400010;

// Modes of Operation (0x6060:00, 8bit) -> 0x60600008
constexpr uint32_t ENTRY_RX_MODES_OF_OP = 0x60600008;

// Target Position (0x607A:00, 32bit) -> 0x607A0020
constexpr uint32_t ENTRY_RX_TARGET_POSITION = 0x607A0020;

// Target Velocity (0x60FF:00, 32bit) -> 0x60FF0020
constexpr uint32_t ENTRY_RX_TARGET_VELOCITY = 0x60FF0020;

// Target Torque (0x6071:00, 16bit) -> 0x60710010
constexpr uint32_t ENTRY_RX_TARGET_TORQUE = 0x60710010;

// Digital Outputs (0x60FE:01, 32bit) -> 0x60FE0120
constexpr uint32_t ENTRY_RX_DIGITAL_OUTPUTS = 0x60FE0120;

// --- [9] TxPDO Mapping Entries (Slave -> Master) ---

// Status Word (0x6041:00, 16bit) -> 0x60410010
constexpr uint32_t ENTRY_TX_STATUS_WORD = 0x60410010;

// Modes of Op Display (0x6061:00, 8bit) -> 0x60610008
constexpr uint32_t ENTRY_TX_MODES_OF_OP_DISP = 0x60610008;

// Actual Position (0x6064:00, 32bit) -> 0x60640020
constexpr uint32_t ENTRY_TX_ACTUAL_POSITION = 0x60640020;

// Actual Velocity (0x606C:00, 32bit) -> 0x606C0020
constexpr uint32_t ENTRY_TX_ACTUAL_VELOCITY = 0x606C0020;

// Actual Torque (0x6077:00, 16bit) -> 0x60770010
constexpr uint32_t ENTRY_TX_ACTUAL_TORQUE = 0x60770010;

// Digital Inputs (0x60FD:00, 32bit) -> 0x60FD0020
constexpr uint32_t ENTRY_TX_DIGITAL_INPUTS = 0x60FD0020;

// Error Code (0x603F:00, 16bit) -> 0x603F0010
constexpr uint32_t ENTRY_TX_ERROR_CODE = 0x603F0010;

// --- [10] Homing Methods (0x6098) ---
constexpr int8_t HM_NEG_LIMIT_SWITCH_AND_INDEX = 1;  // Find index pulse after contacting negative limit switch
constexpr int8_t HM_POS_LIMIT_SWITCH_AND_INDEX = 2;  // Find index pulse after contacting positive limit switch
constexpr int8_t HM_HOME_SWITCH_NEG_AND_INDEX  = 11; // Find index pulse after contacting home switch in negative direction
constexpr int8_t HM_HOME_SWITCH_POS_AND_INDEX  = 7;  // Find index pulse after contacting home switch in positive direction
constexpr int8_t HM_CURRENT_POS_AS_HOME        = 35; // Set current position as home without movement

} // namespace cia402

#endif // CIA402_H
