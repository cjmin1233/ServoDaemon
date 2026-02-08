#include "ecatserver.h"

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
    if (!m_ecatManager->connectMaster()) {
        // connect failed
    }

    // start listening for client connections
    if (m_server->listen(QHostAddress(Config::HOST), Config::PORT)) {
        qDebug() << "[EcatServer::] Server Listening on port" << Config::PORT;

        QObject::connect(m_server, &QTcpServer::newConnection,
                         this, &EcatServer::onServerConnection);
    } else {
        qCritical() << "[EcatServer::] Server Listen Failed" << m_server->errorString();
    }

    QObject::connect(m_timer, &QTimer::timeout,
                     this, &EcatServer::onTimerTick);
    m_timer->start(m_tickCycle);
}

void EcatServer::onServerConnection()
{
    // disconnect previous client if exists
    if (m_currentClient != nullptr) {
        m_currentClient->deleteLater();
    }

    // accept new client connection
    m_currentClient = m_server->nextPendingConnection();

    if (m_currentClient == nullptr) return;

    qDebug() << "[EcatServer::onServerConnection] Client Connected!";

    // connect signals for client socket
    QObject::connect(m_currentClient, &QTcpSocket::readyRead,
                     this, &EcatServer::onClientReadyread);
    QObject::connect(m_currentClient, &QTcpSocket::disconnected,
                     this, &EcatServer::onClientDisconnected);
}

void EcatServer::onClientReadyread()
{
    // read data from client
    QDataStream in(m_currentClient);
    in.setVersion(QDataStream::Qt_6_5);

    // start transaction for safe reading
    in.startTransaction();

    quint32 blockSize;
    float   ratio;

    // read data fields
    in >> blockSize;
    in >> ratio;

    // check if transaction is successful
    if (!in.commitTransaction()) {
        return;
    }

    qDebug() << "Received Ratio:" << ratio;

    // launch servo move
    m_ecatManager->launchServoMove(ratio);
}

void EcatServer::onClientDisconnected()
{
    qDebug() << "[EcatServer::onClientDisconnected] Client Disconnected!";

    m_currentClient = nullptr;
}

void EcatServer::onTimerTick()
{
    // If server is not listening, try to restart listening
    if (!m_server->isListening()) {
        m_server->listen(QHostAddress(Config::HOST), Config::PORT);
        return;
    }

    // If EtherCAT master is not running, try to reconnect
    if (!m_ecatManager->isMasterRunning()) {
        m_ecatManager->reconnectMaster();
        return;
    }

    // qDebug() << "[EcatServer::onTimerTick] is thread terminated :" << m_ecatManager->isThreadTerminated();

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
