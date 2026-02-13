#ifndef SLAVE_H
#define SLAVE_H

extern "C" {
#include "ethercat.h"
}

class Slave {
public:
    Slave(uint16_t slaveId)
        : m_slaveId(slaveId)
    {
    }
    virtual ~Slave() = default;

    virtual void processData() { }

    virtual int  setup() = 0;
    virtual void start() = 0;
    virtual void stop()  = 0;
    // virtual bool isReady() const = 0;

protected:
    uint16_t m_slaveId;

private:
    Slave(const Slave&)            = delete;
    Slave& operator=(const Slave&) = delete;
};

#endif // SLAVE_H
