#ifndef ECATMASTER_H
#define ECATMASTER_H

#include <QObject>
// #include <QVector>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../CommonConfig.h"
#include "slave.h"

extern "C" {
#include "ethercat.h"
}

class ServoL7NH;

class EcatMaster : public QObject {
    Q_OBJECT
public:
    explicit EcatMaster(QObject* parent = nullptr);
    ~EcatMaster()
    {
        if (m_Running) {
            stop();
        }
    }

    bool init(const std::string& ifname);
    bool start();
    void stop();

    ErrorReason processCommand(const Command& cmd);

    ServoStatus   getServoStatus(int slaveId) const;
    const int32_t getServoStrokeMm(int slaveId) const;
    const bool    isRunning() const { return m_Running; }
    // const bool  isServoRunning() const;

    // const bool isThreadTerminated() const
    // {
    //     return !m_Worker.joinable() || !m_ErrorHandler.joinable();
    // }

    bool isAdapterValid(const std::string& ifname);

    // void setTargetPoint(const QVector<int>& point);

signals:
    void allServosArrived();
    // void _Arrived(const QVector<int>& point);

private:
    bool reqOpState();
    void processLoop();
    void ecatCheck();
    void slavesCheck(int wkc, bool forceCheck);
    void monitorLoop();

    ServoL7NH*       getPtrServo(int slaveId);
    const ServoL7NH* getPtrServo(int slaveId) const;

private slots:
    void onServoArrived(uint16_t slaveId);

private:
    std::atomic<bool> m_Running { false };
    std::atomic<bool> m_Initialized { false };

    std::thread m_Worker;
    std::thread m_ErrorHandler;
    std::thread m_Monitor;

    char m_IOmap[4096] = {};

    int              m_ExpectedWKC = 0;
    std::atomic<int> m_CurrentWKC { 0 };
    int              m_CurrentGroup = 0;

    // DC synchronization
    int64_t m_syncOffset = 0;

    std::vector<std::unique_ptr<Slave>> m_Slaves = {};
    std::vector<uint16_t>               m_lastSlaveStates;

    mutable std::mutex   m_ecatMutex;
    std::mutex           m_cmdMutex;
    std::vector<Command> m_cmdQueue;

    std::vector<bool> m_servosArrived = {};

    // QVector<int> m_targetPoint = {};
    // QVector<int> m_curPoint    = {};
};

#endif // ECATMASTER_H
