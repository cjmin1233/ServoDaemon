#include "ecatserver.h"

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
    , m_tickCycle(1000) // 1 sec
{
    QObject::connect(m_server, &QTcpServer::newConnection,
                     this, &EcatServer::onServerConnection);
    QObject::connect(m_timer, &QTimer::timeout,
                     this, &EcatServer::onTimerTick);
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
        qWarning() << "[EcatServer::start] Ecat Master: OFFLINE. Try to reconnect later...";

        // start timer to reconnect
        startTimer();
        return;
    }

    qInfo() << "[EcatServer::start] Ecat Master: ONLINE";

    // 2. Connect server
    if (!m_server->listen(QHostAddress(Config::HOST /*127.0.0.1*/), Config::PORT /*5000*/)) {
        qWarning() << "[EcatServer::start] TCP Server: LISTEN FAILED -" << m_server->errorString();
    } else {
        qInfo() << "[EcatServer::start] TCP Server: LISTENING on port" << Config::PORT;
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
            qWarning() << "[EcatServer::onServerConnection] Kicking old client to accept new connection.";
            m_client->disconnectFromHost();
            m_client->deleteLater();
            m_client = nullptr;
        }

        m_client = newSocket;

        QObject::connect(m_client, &QTcpSocket::readyRead,
                         this, &EcatServer::onClientReadyread);
        QObject::connect(m_client, &QTcpSocket::disconnected,
                         this, &EcatServer::onClientDisconnected);

        qInfo() << "[EcatServer::onServerConnection] New client connected.";
    }
}

void EcatServer::onClientReadyread()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) return;

    QDataStream in(socket);
    in.setVersion(QDataStream::Qt_6_5);

    // start transaction for safe reading
    in.startTransaction();

    quint32 blockSize;
    in >> blockSize;

    Command cmd;
    in >> cmd;

    processCommand(socket, in, cmd);

    // if (m_currentClient == nullptr) return;

    // // read data from client
    // QDataStream in(m_currentClient);
    // in.setVersion(QDataStream::Qt_6_5);

    // // start transaction for safe reading
    // in.startTransaction();

    // // TODO: read command structure
    // quint32 blockSize;
    // in >> blockSize;

    // Command cmd;
    // in >> cmd;

    // processCommand(in, cmd);
}

void EcatServer::onClientDisconnected()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) return;

    if (m_client == socket) {
        m_client = nullptr;
        qInfo() << "[EcatServer::onClientDisconnected] Client disconnected.";
    }

    socket->deleteLater();
}

void EcatServer::onTimerTick()
{
    // If EtherCAT master is not running, try to reconnect
    if (m_ecatManager && !m_ecatManager->isMasterRunning()) {
        qWarning() << "[EcatServer::onTimerTick] Ecat master is not running, try to reconnect...";

        m_ecatManager->reconnectMaster();
        return;
    }

    // If server is not listening, try to restart listening
    if (m_server && !m_server->isListening()) {
        qWarning() << "[EcatServer::onTimerTick] Ecat server is not listening, try to restart...";

        m_server->listen(QHostAddress(Config::HOST), Config::PORT);
        return;
    }

    if (!m_client || m_client->state() != QAbstractSocket::ConnectedState) return;

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

void EcatServer::startTimer()
{
    if (m_timer && !m_timer->isActive()) {
        m_timer->start(m_tickCycle);

        qInfo() << "[EcatServer::start] Ecat server timer is now active.";
    }
}

void EcatServer::processCommand(QTcpSocket* socket, QDataStream& in, const Command& cmd)
{
    if (!in.commitTransaction()) {
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
