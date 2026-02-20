#include "ecatmanager.h"

#include <QDebug>
#include <QThread>

EcatManager::EcatManager(QObject* parent)
    : QObject { parent }
    , m_Master()
    , m_ifname()
{
}

EcatManager::~EcatManager()
{
    disconnectMaster();
}

bool EcatManager::connectMaster()
{
    // try to connect with previous ifname first
    if (!m_ifname.isEmpty() && m_Master.isAdapterValid(m_ifname.toStdString())) {
        return connectMaster(m_ifname);
    }

    qInfo() << "[EcatManager::connectMaster] Failed to connect with previous ifname, or no ifname was set. "
               "Searching for a valid adapter...";
    searchValidAdapter();

    // still no valid ifname found
    if (m_ifname.isEmpty()) {
        return false;
    }

    return connectMaster(m_ifname);
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
    QThread::msleep(100);

    // try to connect again using the main connection logic
    if (connectMaster()) {
        qInfo() << "[EcatManager::reconnectMaster] Reconnection successful!";
    } else {
        qWarning() << "[EcatManager::reconnectMaster] Reconnection failed, will retry later...";
    }
}

void EcatManager::disconnectMaster()
{
    qDebug() << "[EcatManager::disconnectMaster]";

    // stop to terminate threads, reset init state
    m_Master.stop();
}

void EcatManager::processCommand(const Command& cmd)
{
    m_Master.processCommand(cmd);
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
        // qDebug() << "[EcatManager::searchValidAdapter] Found adapter:" << current->name << ", checking validity...";

        if (m_Master.isAdapterValid(current->name)) {
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
