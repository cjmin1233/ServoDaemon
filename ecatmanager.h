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

    bool connectMaster(const QString& ifname);
    void disconnectMaster();

    void launchServoMove(float ratio);
    void setHome();

signals:

private:
    EcatMaster m_Master;
};

#endif // ECATMANAGER_H
