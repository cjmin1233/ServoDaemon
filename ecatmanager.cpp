#include "ecatmanager.h"

#include <QDebug>
#include <QThread>

EcatManager::EcatManager(QObject* parent)
    : QObject { parent }
    , m_Master()
{
}

EcatManager::~EcatManager()
{
    disconnectMaster();
}

bool EcatManager::connectMaster(const QString& ifname)
{
	// init to op state. return false if failed
    if (!m_Master.init(ifname.toStdString())) {
        qWarning() << "[EcatManager::connectMaster] EtherCAT init failed";
        return false;
    }

	// start process loop and error handler threads. return false if failed
    if (!m_Master.start()) {
        qWarning() << "[EcatManager::connectMaster] EtherCAT start failed";
        return false;
    }

    qDebug() << "[EcatManager::connectMaster] EtherCAT connected, slaves:" << ec_slavecount;
    return true;
}

void EcatManager::reconnectMaster(const QString& ifname)
{
    qInfo() << "[EcatManager::reconnectMaster] Attempting to reconnect master...";

	// disconnect first
	disconnectMaster();

    QThread::msleep(1000);

    if (connectMaster(ifname)) {
        qInfo() << "[EcatManager::reconnectMaster] Reconnection successful!";
    } else {
		qWarning() << "[EcatManager::reconnectMaster] Reconnection failed, retry later...";
        return;
    }
}

void EcatManager::disconnectMaster()
{
	// stop to terminate threads, reset init state
    m_Master.stop();
}

void EcatManager::launchServoMove(float ratio)
{
    m_Master.servoMovePosition(ratio);
}

void EcatManager::setHome()
{
    m_Master.setHome();
}
