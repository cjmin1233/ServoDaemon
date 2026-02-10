#ifndef ECATSERVER_H
#define ECATSERVER_H

#include <QObject>

class QTcpServer;
class QTcpSocket;
class QTimer;
class EcatManager;

class EcatServer : public QObject {
    Q_OBJECT
public:
    explicit EcatServer(QObject* parent = nullptr);
    ~EcatServer();

    void start();
    void stop();

private slots:
    void onServerConnection();
    void onClientReadyread();
    void onClientDisconnected();
    void onTimerTick();

private:
    void startTimer();

private:
    QTcpServer* m_server        = nullptr;
    QTcpSocket* m_currentClient = nullptr;

    QTimer* m_timer = nullptr;

    EcatManager* m_ecatManager = nullptr;

    const int m_tickCycle;
};

#endif // ECATSERVER_H
