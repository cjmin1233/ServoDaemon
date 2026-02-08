#ifndef SERVOL7NH_H
#define SERVOL7NH_H

#include "../CommonConfig.h"
#include "slave.h"

bool checkL7NH(int slaveId);

class ServoL7NH : public Slave {
public:
#pragma pack(push, 1) // 메모리 패딩 방지 (중요!)

    // Master -> Slave (RxPDO)
    typedef struct {
        uint16_t control_word;    // 0x6040
        int8_t   mode;            // 0x6060
        int32_t  target_position; // 0x607A
        int32_t  target_velocity; // 0x60FF
        uint32_t digital_outputs; // 0x60FE:01
    } RxPDO;

    // Slave -> Master (TxPDO)
    typedef struct {
        uint16_t status_word;     // 0x6041
        int8_t   mode_disp;       // 0x6061
        int32_t  actual_position; // 0x6064
        int32_t  actual_velocity; // 0x606C
        int16_t  actual_torque;   // 0x6077
        uint32_t digital_inputs;  // 0x60FD
    } TxPDO;

#pragma pack(pop)

public:
    ServoL7NH(uint16_t slaveId)
        : Slave(slaveId)
    {
    }

    virtual void processData() override;
    virtual void start() override;
    virtual void stop() override;

    static bool checkL7NH(int slaveId);
    static int  setupL7NH(uint16 slaveId);

    void setTargetPosition(float ratio);
    void setTargetPosition(int32_t pos);
    void setHome();

    const ServoStatus& getStatus() const { return m_Status; }
    const bool         isRunning() const;

private:
    void stateCheck(RxPDO* rxpdo, const TxPDO* txpdo);
    void processPP(RxPDO* rxpdo, const TxPDO* txpdo);
    void processHM(RxPDO* rxpdo, const TxPDO* txpdo);

    RxPDO*       ptrRxPDO() { return reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs); }
    const TxPDO* ptrTxPDO() const { return reinterpret_cast<const TxPDO*>(ec_slave[m_slaveId].inputs); }

private:
    static constexpr uint32_t s_encoderResolution = 262'144;

    bool m_flagNewSetpoint = false;
    bool m_flagHomingStart = false;

    int m_stateCheckCounter = 0;

    bool     m_isHomingSettling      = false;
    int      m_homingSettlingCounter = 0;
    int      m_homingStableCounter   = 0;
    uint32_t m_posWindow             = 0;

    ServoStatus m_Status = {};
};

#endif // SERVOL7NH_H
