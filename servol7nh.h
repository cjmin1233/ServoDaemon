#ifndef SERVOL7NH_H
#define SERVOL7NH_H

#include "CommonConfig.h"
#include "servood.h"
#include "slave.h"
#include <mutex>

bool checkL7NH(int slaveId);

class ServoL7NH : public Slave {
    Q_OBJECT
public:
#pragma pack(push, 1) // 메모리 패딩 방지 (중요!)

    // Master -> Slave (RxPDO)
    typedef struct {
        uint16_t control_word;    // 0x6040
        int8_t   mode;            // 0x6060
        int32_t  target_position; // 0x607A
        int32_t  target_velocity; // 0x60FF
        int16_t  target_torque;   // 0x6071
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
        uint16_t error_code;      // 0x603F
    } TxPDO;

#pragma pack(pop)

public:
    explicit ServoL7NH(uint16_t slaveId)
        : Slave(slaveId)
    {
    }

    virtual void processData() override;
    virtual void start() override;
    virtual void stop() override;

    void processCommand(const Command& cmd) override;

    static bool checkL7NH(int slaveId);
    static int  setup(uint16 slaveId);
    static bool setupPDO(uint16 slaveId);
    static bool setupPosition(uint16 slaveId);
    static bool setupHoming(uint16 slaveId);
    static bool setupTorque(uint16 slaveId);

    void setTargetPosition(float ratio);
    void setTargetPosition(int32_t pos);
    void setHome();
    // void setTorque(int16_t torque);

    const int32_t getPosLimit() const { return m_posLimit; }
    const int32_t getStrokeMm() const { return m_strokeMm; }

    ServoStatus getStatus() const;
    const bool  isRunning() const;

signals:
    void arrived(uint16_t slaveId);

private:
    void stateCheck(RxPDO* rxpdo, const TxPDO* txpdo);
    void processPP(RxPDO* rxpdo, const TxPDO* txpdo);
    // void processPT(RxPDO* rxpdo, const TxPDO* txpdo);
    void processHM(RxPDO* rxpdo, const TxPDO* txpdo);

    // void settling(RxPDO* rxpdo, const TxPDO* txpdo);

    RxPDO*       ptrRxPDO() { return reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs); }
    const TxPDO* ptrTxPDO() const { return reinterpret_cast<const TxPDO*>(ec_slave[m_slaveId].inputs); }

private:
    bool m_flagNewSetpoint = false;
    bool m_flagHomingStart = false;

    int m_stateCheckCounter = 0;

    // settling variables
    // bool     m_isSettling            = false;
    // int      m_settlingTimeout       = 0;
    // int      m_settlingStableCounter = 0;
    // uint32_t m_posWindow             = 0;

    ServoStatus m_Status = {};

    int32_t m_posLimit    = 0;
    int32_t m_strokeMm    = 0;
    float   m_pulsePerMmf = 0.0f;
    // uint32_t m_pulsePerMm = 0;

    // bool   m_wasWarning = false;
    // int8_t m_lastMode = 0;
    int16_t              m_targetTorque   = 0;
    uint16_t             m_lastEcatState  = 0;
    uint16_t             m_lastStatusWord = 0;
    bool                 m_lastPdoValid   = true;
    servoOD::HomingState m_lastHMState    = static_cast<servoOD::HomingState>(0);

    bool m_lastTargetReached = false;
    int  m_arrivalCount      = 0;

    mutable std::mutex m_mutex;
};

#endif // SERVOL7NH_H
