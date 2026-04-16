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
    qInfo() << "[EcatServer::stop] Stopping server...";

    // stop timer
    m_timer->stop();

    // disconnect clients
    for (QTcpSocket* socket : m_clients) {
        if (socket->state() == QAbstractSocket::ConnectedState) {
            // socket->write("SERVER_SHUTDOWN");
            socket->disconnectFromHost();

            if (socket->waitForDisconnected(1000)) {
                qDebug() << "[EcatServer::stop] Client disconnected safely.";
            }
        }
    }

    qDeleteAll(m_clients);
    m_clients.clear();

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
        QTcpSocket* clientSocket = m_server->nextPendingConnection();

        m_clients.append(clientSocket);

        // connect signals for client socket
        QObject::connect(clientSocket, &QTcpSocket::readyRead,
                         this, &EcatServer::onClientReadyread);
        QObject::connect(clientSocket, &QTcpSocket::disconnected,
                         this, &EcatServer::onClientDisconnected);

        qInfo() << "[EcatServer::onServerConnection] New client connected. Total clients:" << m_clients.size();
    }

    // // disconnect previous client if exists
    // if (m_currentClient != nullptr) {
    //     m_currentClient->deleteLater();
    // }

    // // accept new client connection
    // m_currentClient = m_server->nextPendingConnection();

    // if (m_currentClient == nullptr) {
    //     qWarning() << "[EcatServer::onServerConnection] Client connect failed!";
    //     return;
    // }

    // qDebug() << "[EcatServer::onServerConnection] Client connected!";

    // // connect signals for client socket
    // QObject::connect(m_currentClient, &QTcpSocket::readyRead,
    //                  this, &EcatServer::onClientReadyread);
    // QObject::connect(m_currentClient, &QTcpSocket::disconnected,
    //                  this, &EcatServer::onClientDisconnected);
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

    m_clients.removeAll(socket);
    socket->deleteLater();

    qInfo() << "[EcatServer::onClientDisconnected] Client disconnected. Remaining clients:" << m_clients.size();

    // if (m_currentClient) {
    //     m_currentClient->deleteLater();
    //     m_currentClient = nullptr;
    // }
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

    if (m_clients.isEmpty()) return;

    // Send status of all valid servos to all connected clients
    int totalSlaves = m_ecatManager->getSlaveCount();
    for (int slaveId = 1; slaveId <= totalSlaves; ++slaveId) {
        const ServoStatus& status = m_ecatManager->getServoStatus(slaveId);

        QByteArray block;
        QDataStream out(&block, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_5);

        out << (quint32)0;
        out << (quint32)MessageType::ServoStatusUpdate;
        out << (quint16)slaveId;
        out << status;

        out.device()->seek(0);
        out << (quint32)(block.size() - sizeof(quint32));

        for (QTcpSocket* client : m_clients) {
            if (client->state() == QAbstractSocket::ConnectedState) {
                client->write(block);
            }
        }
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

    QByteArray block;
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
