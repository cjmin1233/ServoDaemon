#include "ecatserver.h"

#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

#include "../CommonConfig.h"
#include "ecatmanager.h"

EcatServer::EcatServer(QObject* parent)
    : QObject { parent }
    , m_server(new QTcpServer(this))
    , m_timer(new QTimer(this))
    , m_ecatManager(new EcatManager(this))
{
    QObject::connect(m_server, &QTcpServer::newConnection, this,
                     &EcatServer::onServerConnection);
    QObject::connect(m_timer, &QTimer::timeout, this, &EcatServer::onTimerTick);

    QObject::connect(m_ecatManager, &EcatManager::allServosArrived,
                     this, &EcatServer::onAllServosArrived);
    // QObject::connect(m_ecatManager, &EcatManager::_Arrived,
    //                  this, &EcatServer::onArrived);
}

EcatServer::~EcatServer()
{
    qDebug() << "[EcatServer::~EcatServer] Close ecat server";

    stop();
}

void EcatServer::start()
{
    qInfo() << "[EcatServer::start] Sequence started. Checking dependencies...";

    // 1. Connect Ecat master
    if (!m_ecatManager->connectMaster()) {
        qWarning() << "[EcatServer::start] Ecat Master: OFFLINE. Try to reconnect "
                      "later...";

        // start timer to reconnect
        startTimer();
        return;
    }

    qInfo() << "[EcatServer::start] Ecat Master: ONLINE";

    // 2. Connect server
    if (!m_server->listen(QHostAddress(Config::HOST /*127.0.0.1*/),
                          Config::PORT /*5000*/)) {
        qWarning() << "[EcatServer::start] TCP Server: LISTEN FAILED -"
                   << m_server->errorString();
    } else {
        qInfo() << "[EcatServer::start] TCP Server: LISTENING on port"
                << Config::PORT;
    }

    // start timer
    startTimer();
}

void EcatServer::stop()
{
    if (!m_timer || !m_timer->isActive()) {
        qWarning() << "[EcatServer::stop] Server already stopped!";
        return;
    }

    qInfo() << "[EcatServer::stop] Stopping server...";

    // stop timer
    m_timer->stop();

    // disconnect client
    if (m_client) {
        if (m_client->state() == QAbstractSocket::ConnectedState) {
            m_client->disconnectFromHost();
            if (m_client->waitForDisconnected(1000)) {
                qDebug() << "[EcatServer::stop] Client disconnected safely.";
            }
        }
        m_client->deleteLater();
        m_client = nullptr;
    }

    // close server
    if (m_server->isListening()) {
        m_server->close();
        qInfo() << "[EcatServer::stop] TCP Server is now closed.";
    }

    // disconnect EtherCAT master
    m_ecatManager->disconnectMaster();
}

void EcatServer::onServerConnection()
{
    while (m_server->hasPendingConnections()) {
        QTcpSocket* newSocket = m_server->nextPendingConnection();

        if (m_client) {
            qWarning() << "[EcatServer::onServerConnection] Kicking old client to "
                          "accept new connection.";
            m_client->disconnectFromHost();
            m_client->deleteLater();
            m_client = nullptr;
        }

        m_client         = newSocket;
        m_lastPacketTime = QDateTime::currentMSecsSinceEpoch();

        QObject::connect(m_client, &QTcpSocket::readyRead, this,
                         &EcatServer::onClientReadyread);
        QObject::connect(m_client, &QTcpSocket::disconnected, this,
                         &EcatServer::onClientDisconnected);

        qInfo() << "[EcatServer::onServerConnection] New client connected.";

        SystemInitData initData;
        initData.totalServos = m_ecatManager->getSlaveCount();

        for (quint16 i = 1; i <= initData.totalServos; ++i) {
            ServoInitData servoData;
            servoData.slaveId = i;

            servoData.minPosition = 0;
            servoData.maxPosition = m_ecatManager->getServoStrokeMm(i);

            initData.servos.append(servoData);
        }

        QByteArray  block;
        QDataStream out(&block, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_5);

        out << (quint32)0;
        out << (quint32)MessageType::SystemInitData;
        out << initData;

        out.device()->seek(0);
        out << (quint32)(block.size() - sizeof(quint32));

        m_client->write(block);
        m_client->flush();
    }
}

void EcatServer::onClientReadyread()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket)
        return;

    // Update watchdog timestamp on ANY data received
    m_lastPacketTime = QDateTime::currentMSecsSinceEpoch();

    QDataStream in(socket);
    in.setVersion(QDataStream::Qt_6_5);

    // process data from client
    while (socket->bytesAvailable() > sizeof(quint32)) {
        in.startTransaction();

        quint32 blockSize;
        in >> blockSize;

        Command cmd;
        in >> cmd;

        if (cmd.cmdType == CommandType::MovePoint) {
            int slaveCount = m_ecatManager->getSlaveCount();
            // QVector<int> point(slaveCount);

            for (int i = 0; i < slaveCount; ++i) {
                int p;
                in >> p;

                // point[i] = p;

                processCommand(socket, { (quint16)(i + 1), CommandType::MovePosition, p });
            }

            // m_ecatManager->setTargetPoint(point);

            if (!in.commitTransaction()) {
                break;
            }
        } else if (in.commitTransaction()) {
            processCommand(socket, cmd);
        } else {
            break;
        }
    }
}

void EcatServer::onClientDisconnected()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket)
        return;

    if (m_client == socket) {
        m_client = nullptr;
        qInfo() << "[EcatServer::onClientDisconnected] Client disconnected.";
    }

    socket->deleteLater();

    // // Stop all servos
    // int totalSlaves = m_ecatManager->getSlaveCount();
    // for (int slaveId = 1; slaveId <= totalSlaves; ++slaveId) {
    //     Command stopCmd;
    //     stopCmd.slaveId = slaveId;
    //     stopCmd.cmdType = CommandType::StopServo;
    //     m_ecatManager->processCommand(stopCmd);
    // }
}

void EcatServer::onTimerTick()
{
    // If EtherCAT master is not running, try to reconnect
    bool isRunning = m_ecatManager->isMasterRunning();

    if (!isRunning) {
        if (m_lastMasterRunningState) {
            qWarning() << "[EcatServer::onTimerTick] Ecat master is not running, try "
                          "to reconnect...";
            m_lastMasterRunningState = false;
        }
        m_ecatManager->reconnectMaster();
        return;
    }

    if (!m_lastMasterRunningState) {
        qInfo() << "[EcatServer::onTimerTick] Ecat master is ONLINE";
        m_lastMasterRunningState = true;
    }

    // If server is not listening, try to restart listening
    if (m_server && !m_server->isListening()) {
        qWarning() << "[EcatServer::onTimerTick] Ecat server is not listening, try "
                      "to restart...";

        m_server->listen(QHostAddress(Config::HOST), Config::PORT);
        return;
    }

    if (!m_client || m_client->state() != QAbstractSocket::ConnectedState)
        return;

    // Watchdog check
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - m_lastPacketTime > m_watchdogTimeoutMs) {
        qWarning()
            << "[EcatServer::onTimerTick] Watchdog timeout! Halting all servos.";

        // // Stop all servos
        // int totalSlaves = m_ecatManager->getSlaveCount();
        // for (int slaveId = 1; slaveId <= totalSlaves; ++slaveId) {
        //     Command stopCmd;
        //     stopCmd.slaveId = slaveId;
        //     stopCmd.cmdType = CommandType::StopServo;
        //     m_ecatManager->processCommand(stopCmd);
        // }

        // Disconnect client as penalty
        m_client->disconnectFromHost();
        return;
    }

    // Send status of all valid servos to the connected client
    int totalSlaves = m_ecatManager->getSlaveCount();
    for (int slaveId = 1; slaveId <= totalSlaves; ++slaveId) {
        ServoStatus status = m_ecatManager->getServoStatus(slaveId);

        QByteArray  block;
        QDataStream out(&block, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_5);

        out << (quint32)0;
        out << (quint32)MessageType::ServoStatusUpdate;
        out << (quint16)slaveId;
        out << status;

        out.device()->seek(0);
        out << (quint32)(block.size() - sizeof(quint32));

        m_client->write(block);
    }
}

void EcatServer::onAllServosArrived()
{
    if (!m_client || m_client->state() != QAbstractSocket::ConnectedState)
        return;

    QByteArray  block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_5);

    out << (quint32)0;
    out << (quint32)MessageType::ServoArriveAlarm;

    out.device()->seek(0);
    out << (quint32)(block.size() - sizeof(quint32));

    m_client->write(block);
    m_client->flush();
}

// void EcatServer::onArrived(const QVector<int>& point)
// {
//     if (!m_client || m_client->state() != QAbstractSocket::ConnectedState)
//         return;

//     QByteArray  block;
//     QDataStream out(&block, QIODevice::WriteOnly);
//     out.setVersion(QDataStream::Qt_6_5);

//     out << (quint32)0;
//     out << (quint32)MessageType::ServoArriveAlarm;

//     for (int i = 0; i < point.size(); ++i) {
//         out << point[i];
//     }

//     out.device()->seek(0);
//     out << (quint32)(block.size() - sizeof(quint32));

//     m_client->write(block);
//     m_client->flush();
// }

void EcatServer::startTimer()
{
    if (m_timer && !m_timer->isActive()) {
        m_timer->start(m_tickCycleMs);

        qInfo() << "[EcatServer::start] Ecat server timer is now active.";
    }
}

void EcatServer::processCommand(QTcpSocket* socket, const Command& cmd)
{
    // If it's just a heartbeat, we are done (timestamp already updated in
    // onClientReadyread)
    if (cmd.cmdType == CommandType::Heartbeat) {
        return;
    }

    ErrorReason result = m_ecatManager->processCommand(cmd);

    CommandResponse response;
    response.slaveId = cmd.slaveId;
    response.cmdType = cmd.cmdType;

    if (result == ErrorReason::None) {
        response.status = ResponseStatus::ACK;
    } else {
        response.status = ResponseStatus::NACK;
        response.reason = result;
    }

    QByteArray  block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_5);

    out << (quint32)0; // Size placeholder
    out << (quint32)MessageType::CommandResponse;
    out << response;

    out.device()->seek(0);
    out << (quint32)(block.size() - sizeof(quint32)); // Write actual size

    socket->write(block);
    socket->flush();
}
