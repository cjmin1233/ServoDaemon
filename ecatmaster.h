#ifndef ECATMASTER_H
#define ECATMASTER_H

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "../CommonConfig.h"
#include "slave.h"

extern "C" {
#include "ethercat.h"
}

class ServoL7NH;

class EcatMaster {
public:
    // Singleton
    static EcatMaster& instance()
    {
        static EcatMaster instance;
        return instance;
    }

    // EcatMaster() = default;
    ~EcatMaster()
    {
        if (m_Running) {
            stop();
        }
    }

    bool init(const std::string& ifname);
    int  setupSlave(uint16_t slaveId) { return m_Slaves[slaveId]->setup(); }
    bool start();
    void stop();

    void servoMovePosition(float ratio);
    void setHome();
    void setTorque();

    const ServoStatus& getServoStatus(int slaveId) const;
    const bool         isRunning() const { return m_Running; }
    const bool         isServoRunning() const;

    const bool isThreadTerminated() const
    {
        return !m_Worker.joinable() || !m_ErrorHandler.joinable();
    }

    bool isAdapterValid(const std::string& ifname);

private:
    void processLoop();
    bool reqOpState();
    void ecatCheck();
    void slavesCheck();

    ServoL7NH*       getPtrServo();
    const ServoL7NH* getPtrServo() const;

private:
    std::atomic<bool> m_Running { false };
    std::atomic<bool> m_Initialized { false };

    std::thread m_Worker;
    std::thread m_ErrorHandler;

    char m_IOmap[4096] = {};

    int              m_ExpectedWKC = 0;
    std::atomic<int> m_CurrentWKC { 0 };
    int              m_CurrentGroup = 0;

    std::vector<std::unique_ptr<Slave>> m_Slaves = {};

    int m_ServoId = 0;

private:
    EcatMaster()                             = default;
    EcatMaster(const EcatMaster&)            = delete;
    EcatMaster& operator=(const EcatMaster&) = delete;
};

#endif // ECATMASTER_H
