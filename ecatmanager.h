#ifndef ECATMANAGER_H
#define ECATMANAGER_H

#include <QObject>
// #include <QVector>

#include "../CommonConfig.h"
#include "ecatmaster.h"

class EcatManager : public QObject {
    Q_OBJECT
public:
    explicit EcatManager(QObject* parent = nullptr);
    ~EcatManager();

    const int   getSlaveCount() const { return ec_slavecount; }
    ServoStatus getServoStatus(int slaveId) const { return m_Master->getServoStatus(slaveId); }
    const int   getServoStrokeMm(int slaveId) const { return m_Master->getServoStrokeMm(slaveId); }
    const bool  isMasterRunning() const { return m_Master->isRunning(); }

    bool connectMaster();
    bool connectMaster(const QString& ifname);
    void reconnectMaster();
    void disconnectMaster();

    ErrorReason processCommand(const Command& cmd) { return m_Master->processCommand(cmd); }

    // void setTargetPoint(const QVector<int>& point) { m_Master->setTargetPoint(point); }

signals:
    void allServosArrived();
    // void _Arrived(const QVector<int>& point);

private:
    void searchValidAdapter();

private slots:
    void onAllServosArrived() { emit allServosArrived(); }
    // void onArrived(const QVector<int>& point) { emit _Arrived(point); }

private:
    EcatMaster* m_Master = nullptr;
    QString     m_ifname;
    bool        m_isConnecting = false;
};

#endif // ECATMANAGER_H
