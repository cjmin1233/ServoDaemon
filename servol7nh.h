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
        uint32_t position_window; // 0x6067
        uint32_t digital_inputs;  // 0x60FD
        uint16_t error_code;      // 0x603F
    } TxPDO;

#pragma pack(pop)

public:
    ServoL7NH(uint16_t slaveId);

    virtual void processData() override;
    int          setup() override;
    virtual void start() override;
    virtual void stop() override;

    static bool checkL7NH(int slaveId);

    int setupPDO();
    int setupPos();
    int setupHome();
    int setupTorque();

    // static int setup(uint16 slaveId);
    // static int setupPDO(uint16 slaveId);
    // static int setupPosition(uint16 slaveId);
    // static int setupHoming(uint16 slaveId);
    // static int setupTorque(uint16 slaveId);

    void setNone();
    void setTargetPosition(float ratio);
    void setTargetPosition(int32_t pos);
    void setHome();
    void setTorque(int16_t torque);

    const ServoStatus& getStatus() const { return m_Status; }
    const bool         isRunning() const;

private:
    void stateCheck(RxPDO* rxpdo, const TxPDO* txpdo);
    void processPP(RxPDO* rxpdo, const TxPDO* txpdo);
    void processPT(RxPDO* rxpdo, const TxPDO* txpdo);
    void processHM(RxPDO* rxpdo, const TxPDO* txpdo);

    void settling(RxPDO* rxpdo, const TxPDO* txpdo);

    const bool isInPosition(RxPDO* rxpdo, const TxPDO* txpdo) const;

    RxPDO*       ptrRxPDO() { return reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs); }
    const TxPDO* ptrTxPDO() const { return reinterpret_cast<const TxPDO*>(ec_slave[m_slaveId].inputs); }

    const uint32_t getEncoderPPR() const { return m_encoderPPR; }
    // void           setEncoderPPR(uint32_t ppr);

private:
    // static constexpr uint32_t s_encoderResolution = 262'144;

    bool m_flagNewSetpoint = false;
    bool m_flagHomingStart = false;

    int m_stateCheckCounter = 0;

    // settling variables
    bool m_isSettling            = false;
    int  m_settlingTimeout       = 0;
    int  m_settlingStableCounter = 0;
    // uint32_t m_posWindow             = 0;

    ServoStatus m_Status = {};

    uint32_t m_encoderPPR = 0;
};

#endif // SERVOL7NH_H
