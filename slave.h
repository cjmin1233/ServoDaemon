#ifndef SLAVE_H
#define SLAVE_H

extern "C" {
#include "ethercat.h"
}

#include "../CommonConfig.h"

class Slave {
public:
    Slave(uint16_t slaveId)
        : m_slaveId(slaveId)
    {
    }
    virtual ~Slave() = default;

    virtual void processData() { }

    virtual void start() = 0;
    virtual void stop()  = 0;
    // virtual bool isReady() const = 0;

    virtual void processCommand(const Command& cmd) = 0;

    const int16_t getOverloadRatio() const { return m_overloadRatio; }
    void          setOverloadRatio(int16_t overload) { m_overloadRatio = overload; }

protected:
    uint16_t m_slaveId;

private:
    int16_t m_overloadRatio = 0;

private:
    Slave(const Slave&)            = delete;
    Slave& operator=(const Slave&) = delete;
};

#endif // SLAVE_H
