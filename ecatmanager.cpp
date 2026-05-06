#include "ecatmanager.h"

#include <QDebug>
#include <QThread>
#include <QTimer>

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
    if (m_isConnecting) {
        qDebug() << "[EcatManager::connectMaster] Already attempting to connect, skipping...";
        return false;
    }

    if (m_Master.isRunning()) {
        qDebug() << "[EcatManager::connectMaster] Master is running...";
        return true;
    }

    m_isConnecting = true;

    // try to connect with previous ifname first
    if (!m_ifname.isEmpty() && m_Master.isAdapterValid(m_ifname.toStdString())) {
        bool ok        = connectMaster(m_ifname);
        m_isConnecting = false;
        return ok;
    }

    qInfo() << "[EcatManager::connectMaster] Failed to connect with previous ifname, or no ifname was set. "
               "Searching for a valid adapter...";
    searchValidAdapter();

    // still no valid ifname found
    if (m_ifname.isEmpty()) {
        m_isConnecting = false;
        return false;
    }

    bool ok        = connectMaster(m_ifname);
    m_isConnecting = false;
    return ok;
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

    // wait for a moment to allow hardware/drivers to settle without blocking the event loop
    QTimer::singleShot(100, this, [this]() {
        // try to connect again using the main connection logic
        if (connectMaster()) {
            qInfo() << "[EcatManager::reconnectMaster] Reconnection successful!";
        } else {
            qWarning() << "[EcatManager::reconnectMaster] Reconnection failed, will retry later...";
        }
    });
}

void EcatManager::disconnectMaster()
{
    qDebug() << "[EcatManager::disconnectMaster]";

    // Reset connecting flag first to allow new attempts
    m_isConnecting = false;

    // stop to terminate threads, reset init state
    m_Master.stop();
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
