#include "windowsservice.h"
#include <QCoreApplication>

#include <iostream>
// #include <QDir>
// #include <QFile>

// #include "../CommonConfig.h"
// #include "ecatserver.h"

#if 0
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
#endif

int main(int argc, char* argv[])
{
    QString        serviceName = "ServoServiceDemo";
    WindowsService service(serviceName);

    if (argc > 1) {
        QString arg(argv[1]);
        if (arg == "-install" || arg == "--install") {
            // Need a QCoreApplication to resolve applicationFilePath for installation
            QCoreApplication a(argc, argv);
            if (service.install()) {
                std::cout << "Service installed successfully.\n";
                return 0;
            } else {
                std::cout << "Failed to install service.\n";
                return 1;
            }
        } else if (arg == "-uninstall" || arg == "--uninstall") {
            if (service.uninstall()) {
                std::cout << "Service uninstalled successfully.\n";
                return 0;
            } else {
                std::cout << "Failed to uninstall service.\n";
                return 1;
            }
        }
    }

    if (!service.run()) {
        std::cout << "This application is a Windows Service and must be run via the Service Control Manager.\n";
        std::cout << "To install it, run: " << argv[0] << " -install\n";
        return 1;
    }

    return 0;

    // #ifndef QT_DEBUG
    //     // install custom message handler for logging
    //     qInstallMessageHandler(myMessageOutput);
    // #endif

    //     QCoreApplication a(argc, argv);

    //     qDebug() << "---------- Servo Daemon Started ----------";

    //     EcatServer* server = new EcatServer(&a);
    //     server->start();

    //     return a.exec();
}
