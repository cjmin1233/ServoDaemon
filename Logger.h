#ifndef LOGGER_H
#define LOGGER_H

#include <QtGlobal>
#include <QString>

namespace Logger {
    /**
     * @brief spdlog 로깅 시스템 초기화 (비동기, 로테이션 Sink 설정)
     */
    void init();

    /**
     * @brief Qt 메시지 핸들러 브릿지
     * qInfo(), qDebug() 등으로 출력되는 내용을 spdlog로 전달합니다.
     */
    void qtMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);
}

#endif // LOGGER_H
