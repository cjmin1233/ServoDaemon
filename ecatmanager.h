#ifndef ECATMANAGER_H
#define ECATMANAGER_H

#include <QObject>

#include "../CommonConfig.h"
#include "ecatmaster.h"

class EcatManager : public QObject {
    Q_OBJECT
public:
    explicit EcatManager(QObject* parent = nullptr);
    ~EcatManager();

    const int          getSlaveCount() const { return ec_slavecount; }
    const ServoStatus& getServoStatus(int slaveId) const { return m_Master.getServoStatus(slaveId); }
    const bool         isMasterRunning() const { return m_Master.isRunning(); }

    bool connectMaster();
    bool connectMaster(const QString& ifname);
    void reconnectMaster();
    void disconnectMaster();

    void launchServoMove(float ratio);
    void setHome();

private:
    void searchValidAdapter();

private:
    EcatMaster m_Master;

    QString m_ifname;
};

#endif // ECATMANAGER_H
