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
    qInfo() << "[EcatServer] Stopping server...";

    // stop timer
    m_timer->stop();

    // disconnect client
    if (m_currentClient) {
        m_currentClient->disconnectFromHost();

        // automatically call onClientDisconnected... no need to delete client
    }

    // close server
    if (m_server->isListening()) {
        m_server->close();
    }

    // disconnect EtherCAT master
    m_ecatManager->disconnectMaster();
}

void EcatServer::onServerConnection()
{
    // disconnect previous client if exists
    if (m_currentClient != nullptr) {
        m_currentClient->deleteLater();
    }

    // accept new client connection
    m_currentClient = m_server->nextPendingConnection();

    if (m_currentClient == nullptr) {
        qWarning() << "[EcatServer::onServerConnection] Client connect failed!";
        return;
    }

    qDebug() << "[EcatServer::onServerConnection] Client connected!";

    // connect signals for client socket
    QObject::connect(m_currentClient, &QTcpSocket::readyRead,
                     this, &EcatServer::onClientReadyread);
    QObject::connect(m_currentClient, &QTcpSocket::disconnected,
                     this, &EcatServer::onClientDisconnected);
}

void EcatServer::onClientReadyread()
{
    if (m_currentClient == nullptr) return;

    // read data from client
    QDataStream in(m_currentClient);
    in.setVersion(QDataStream::Qt_6_5);

    // start transaction for safe reading
    in.startTransaction();

    // TODO: read command structure
    quint32 blockSize;
    in >> blockSize;

    Command cmd;
    in >> cmd;

    processCommand(in, cmd);
}

void EcatServer::onClientDisconnected()
{
    qDebug() << "[EcatServer::onClientDisconnected] Client Disconnected!";

    if (m_currentClient) {
        m_currentClient->deleteLater();
        m_currentClient = nullptr;
    }
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

    // If there is a connected client, send servo status
    // TODO: send status to all connected clients
    if (m_currentClient && m_currentClient->state() == QAbstractSocket::ConnectedState) {
        int servoId = 1; // temporary

        const ServoStatus& status = m_ecatManager->getServoStatus(servoId);
        QByteArray         block;
        QDataStream        out(&block, QIODevice::WriteOnly);

        out.setVersion(QDataStream::Qt_6_5);

        // 1. size placeholder, actual size will be written later
        out << (quint32)0;
        out << status.position << status.velocity;

        // 2. go back and write the actual size
        out.device()->seek(0);
        out << (quint32)(block.size() - sizeof(quint32));

        // 3. send the data block to client
        m_currentClient->write(block);
        // m_currentClient->write("server tick");

        // wait until all data is written
        m_currentClient->waitForBytesWritten();
        m_currentClient->flush();
    }
}

void EcatServer::startTimer()
{
    if (m_timer && !m_timer->isActive()) {
        m_timer->start(m_tickCycle);

        qInfo() << "[EcatServer::start] Ecat server timer is now active.";
    }
}

void EcatServer::processCommand(QDataStream& in, const Command& cmd)
{
    if (!in.commitTransaction()) {
        return;
    }

    m_ecatManager->processCommand(cmd);
}
