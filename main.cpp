#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QSharedMemory>
#include <QSystemSemaphore>

#include "../CommonConfig.h"
#include "ecatserver.h"

/// <summary>
/// Qt custom message handler for logging
/// </summary>
/// <param name="type"> Type of message </param>
/// <param name="context"> Message context </param>
/// <param name="msg"> Message content </param>
void myMessageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    // create logs directory if not exists
    QString logDirPath = QCoreApplication::applicationDirPath() + "/logs";
    QDir    logDir(logDirPath);
    if (!logDir.exists()) logDir.mkpath(".");

    // define log file path based on current date
    QString dateString  = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    QString logFilePath = logDirPath + QString("/%1_log.txt").arg(dateString);

    QFile outFile(logFilePath);
    if (outFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
        QTextStream ts(&outFile);
        QString     timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

        // determine message type string
        QString typeStr = "INFO ";
        if (type == QtCriticalMsg || type == QtFatalMsg) typeStr = "ERROR";

        ts << "[" << timeStr << "] [" << typeStr << "] " << msg << Qt::endl;
        outFile.close();
    }
}

int main(int argc, char* argv[])
{
#ifndef QT_DEBUG
    // install custom message handler for logging
    qInstallMessageHandler(myMessageOutput);
#endif

    QCoreApplication a(argc, argv);

    // // Check for another instance using QSharedMemory
    // QSharedMemory sharedMemory("ServoDaemon_Instance");
    // if (!sharedMemory.create(1)) {
    //     qCritical() << "Another instance of ServoDaemon is already running";
    //     return -1; // Exit if another instance is found
    // }

    qDebug() << "---------- Servo Daemon Started ----------";

    EcatServer* server = new EcatServer(&a);
    server->start();

    return a.exec();
}
