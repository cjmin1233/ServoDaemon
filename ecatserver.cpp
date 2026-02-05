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
{
    const QString ifname = "\\Device\\NPF_{F80FCB79-A945-4A5A-BD77-B5076391E949}";

    if (!m_ecatManager->connectMaster(ifname)) {
        // connect failed
    }

    if (m_server->listen(QHostAddress(Config::HOST), Config::PORT)) {
        qDebug() << "[EcatServer::] Server Listening on port" << Config::PORT;

        QObject::connect(m_server, &QTcpServer::newConnection,
                         this, &EcatServer::onServerConnection);
    } else {
        qCritical() << "[EcatServer::] Server Listen Failed" << m_server->errorString();
    }

    QObject::connect(m_timer, &QTimer::timeout,
                     this, &EcatServer::onTimerTick);
    m_timer->start(1000);
}

void EcatServer::onServerConnection()
{
    if (m_currentClient != nullptr) {
        m_currentClient->deleteLater();
    }

    m_currentClient = m_server->nextPendingConnection();

    if (m_currentClient == nullptr) return;

    qDebug() << "[EcatServer::onServerConnection] Client Connected!";

    QObject::connect(m_currentClient, &QTcpSocket::readyRead,
                     this, &EcatServer::onClientReadyread);
    QObject::connect(m_currentClient, &QTcpSocket::disconnected,
                     this, &EcatServer::onClientDisconnected);
}

void EcatServer::onClientReadyread()
{
    QDataStream in(m_currentClient);
    in.setVersion(QDataStream::Qt_6_5);

    in.startTransaction();

    quint32 blockSize;
    float   ratio;

    in >> blockSize;
    in >> ratio; // float 타입으로 읽기

    if (!in.commitTransaction()) {
        return;
    }

    qDebug() << "Received Ratio:" << ratio;

    // EtherCAT 마스터 객체에 전달
    m_ecatManager->launchServoMove(ratio);
    // ecatMaster->servoMovePosition(targetPos);

    // QByteArray data = m_currentClient->readAll();

    // qDebug() << "[EcatServer::onClientReadyread] Received Command:" << data;

    // // command switch case
    // if (data == "MOTOR_ON") {
    //     qDebug() << "[EcatServer::onClientReadyread] Motor on...";
    // }
}

void EcatServer::onClientDisconnected()
{
    qDebug() << "[EcatServer::onClientDisconnected] Client Disconnected!";

    m_currentClient = nullptr;
}

void EcatServer::onTimerTick()
{
    const QString& ifname = "\\Device\\NPF_{F80FCB79-A945-4A5A-BD77-B5076391E949}";

    if (!m_ecatManager->isMasterRunning()) {
        m_ecatManager->reconnectMaster(ifname);
        return;
    }

    if (m_currentClient && m_currentClient->state() == QAbstractSocket::ConnectedState) {
        int servoId = 1; // temporary

        const ServoStatus& status = m_ecatManager->getServoStatus(servoId);
        QByteArray         block;
        QDataStream        out(&block, QIODevice::WriteOnly);

        out.setVersion(QDataStream::Qt_6_5);

        // 1. 헤더 자리를 비워두고 데이터 쓰기
        out << (quint32)0;
        out << status.position << status.velocity;

        // 2. 전체 크기를 계산해서 맨 앞으로 이동 후 덮어쓰기
        out.device()->seek(0);
        out << (quint32)(block.size() - sizeof(quint32));

        m_currentClient->write(block);

        // m_currentClient->write("server tick");
        m_currentClient->waitForBytesWritten();
        m_currentClient->flush();
    }
}
