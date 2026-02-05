#include <QCoreApplication>
#include <QDir>
#include <QFile>

#include "../CommonConfig.h"
#include "ecatserver.h"

// [1] 날짜별 로그 파일 저장 함수
void myMessageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    // 실행 파일 경로 하위의 logs 폴더
    QString logDirPath = QCoreApplication::applicationDirPath() + "/logs";
    QDir    logDir(logDirPath);
    if (!logDir.exists()) logDir.mkpath(".");

    // 날짜별 파일명 (예: 2024-05-20_log.txt)
    QString dateString  = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    QString logFilePath = logDirPath + QString("/%1_log.txt").arg(dateString);

    QFile outFile(logFilePath);
    if (outFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
        QTextStream ts(&outFile);
        QString     timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

        // 로그 타입 표시
        QString typeStr = "INFO ";
        if (type == QtCriticalMsg || type == QtFatalMsg) typeStr = "ERROR";

        ts << "[" << timeStr << "] [" << typeStr << "] " << msg << Qt::endl;
        outFile.close();
    }
}

int main(int argc, char* argv[])
{
    // 핸들러 등록 (이제 모든 qDebug는 파일로 감)
    qInstallMessageHandler(myMessageOutput);

    QCoreApplication a(argc, argv);

    qDebug() << "--- Servo Daemon Started ---";

    auto* server = new EcatServer(&a);

    return a.exec();
}
