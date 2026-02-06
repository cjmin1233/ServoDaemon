#include "ecatmanager.h"

#include <QDebug>
#include <QThread>

EcatManager::EcatManager(QObject* parent)
    : QObject { parent }
    , m_Master()
    , m_ifname("") // TODO: Get from config
// , m_ifname("\\Device\\NPF_{F80FCB79-A945-4A5A-BD77-B5076391E949}") // TODO: Get from config
{
}

EcatManager::~EcatManager()
{
    disconnectMaster();
}

bool EcatManager::connectMaster()
{
    // try to connect with previous ifname first
    if (!m_ifname.isEmpty() && connectMaster(m_ifname)) {
        return true;
    }

    qInfo() << "[EcatManager::connectMaster] Failed to connect with previous ifname, or no ifname was set. "
               "Searching for a valid adapter...";
    searchValidAdapter();

    // still no valid ifname found
    if (m_ifname.isEmpty()) {
        return false;
    }

	// start process loop and error handler threads. return false if failed
    if (!m_Master.start()) {
        qWarning() << "[EcatManager::connectMaster] EtherCAT start failed";
        return false;
    }

    return true;
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

void EcatManager::reconnectMaster()
{
    qInfo() << "[EcatManager::reconnectMaster] Attempting to reconnect master...";

    // disconnect first to ensure a clean state
    disconnectMaster();

    // wait for a moment to allow hardware/drivers to settle
    QThread::msleep(1000);

    // try to connect again using the main connection logic
    if (connectMaster()) {
        qInfo() << "[EcatManager::reconnectMaster] Reconnection successful!";
    } else {
        qWarning() << "[EcatManager::reconnectMaster] Reconnection failed, will retry later...";
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

// search for a valid EtherCAT adapter and update m_ifname
void EcatManager::searchValidAdapter()
{
	// reset ifname
    m_ifname = "";

    qInfo() << "[EcatManager::searchValidAdapter] Searching for a valid EtherCAT adapter...";

    ec_adaptert* adapter = ec_find_adapters();
    ec_adaptert* current = adapter;
    while (current != nullptr) {
        qDebug() << "[EcatManager::searchValidAdapter] Found adapter:" << current->name << ", checking validity...";

        // Try to init master on this adapter to check if it's valid
        if (m_Master.init(current->name)) {
            // If init succeeds, it's a valid adapter with slaves
            m_ifname = current->name;
            qInfo() << "[EcatManager::searchValidAdapter] Found valid adapter, updated ifname to:" << m_ifname;

            // Clean up immediately after check
            ec_free_adapters(adapter);
            return;
        }

        // Move to next adapter
        current = current->next;
    }

    qWarning() << "[EcatManager::searchValidAdapter] Could not find any valid EtherCAT adapter.";
    ec_free_adapters(adapter);
    return;
}
