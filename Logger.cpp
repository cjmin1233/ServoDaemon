#include "Logger.h"

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <QCoreApplication>
#include <QDir>
#include <vector>

namespace Logger {

void init() {
    try {
        // 1. 로그 디렉토리 생성 및 경로 설정
        QString logDirPath = QCoreApplication::applicationDirPath() + "/logs";
        QDir().mkpath(logDirPath);
        
        // 로그 파일 경로 설정
        std::string logFilePath = (logDirPath + "/servodaemon.log").toStdString();

        // 2. 비동기 처리를 위한 스레드 풀 초기화 (큐 크기 8192, 스레드 1개)
        spdlog::init_thread_pool(8192, 1);

        // 3. 싱크(Sink) 구성
        std::vector<spdlog::sink_ptr> sinks;
        
        // [Sink 1] 콘솔 출력 (디버깅용)
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sinks.push_back(console_sink);
        
        // [Sink 2] 로테이션 파일 저장 (최대 5MB, 최대 5개 파일 보관)
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            logFilePath, 1024 * 1024 * 5, 5);
        sinks.push_back(rotating_sink);

        // 4. 비동기 로거 생성
        auto logger = std::make_shared<spdlog::async_logger>(
            "servodaemon", 
            sinks.begin(), 
            sinks.end(), 
            spdlog::thread_pool(), 
            spdlog::async_overflow_policy::block
        );
        
        // 5. 전역 설정
        spdlog::set_default_logger(logger);
        
        // 로그 포맷: [날짜 시:분:초.밀리초] [레벨] [메시지]
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
        
#ifdef QT_DEBUG
        spdlog::set_level(spdlog::level::debug);
#else
        spdlog::set_level(spdlog::level::info);
#endif

        // 강제 플러시 주기 (매 3초)
        spdlog::flush_every(std::chrono::seconds(3));

        spdlog::info("spdlog initialization successful (Async + Rotating)");

    } catch (const spdlog::spdlog_ex& ex) {
        fprintf(stderr, "Log initialization failed: %s\n", ex.what());
    }
}

void qtMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    (void)context; // 사용되지 않는 파라미터 경고 방지
    std::string message = msg.toStdString();
    
    switch (type) {
    case QtDebugMsg:
        spdlog::debug(message);
        break;
    case QtInfoMsg:
        spdlog::info(message);
        break;
    case QtWarningMsg:
        spdlog::warn(message);
        break;
    case QtCriticalMsg:
        spdlog::error(message);
        break;
    case QtFatalMsg:
        spdlog::critical(message);
        break;
    }
}

} // namespace Logger
