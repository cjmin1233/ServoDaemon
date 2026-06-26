#include "windowsservice.h"
#include "ecatserver.h"
#include "Logger.h"

#include <QCoreApplication>
#include <QDebug>

#include "configloader.h" // For config initialization

// 서비스 관리를 위한 정적 멤버 변수 초기화
QString               WindowsService::m_serviceName   = "";
SERVICE_STATUS_HANDLE WindowsService::m_statusHandle  = nullptr;
SERVICE_STATUS        WindowsService::m_serviceStatus = { 0 };


WindowsService::WindowsService(const QString& serviceName)
{
    m_serviceName = serviceName;
}

/**
 * @brief 서비스를 Windows 시스템에 등록(설치)
 */
bool WindowsService::install()
{
    SC_HANDLE scm = OpenSCManager(nullptr, nullptr, SC_MANAGER_ALL_ACCESS);
    if (!scm) {
        qCritical() << "Failed to open Service Control Manager";
        return false;
    }

    QString appPath = QCoreApplication::applicationFilePath();
    appPath.replace("/", "\\");
    appPath = "\"" + appPath + "\" --service \"" + m_serviceName + "\"";

    const std::wstring wideName = m_serviceName.toStdWString();

    SC_HANDLE service = CreateServiceW(
        scm,
        wideName.c_str(),
        wideName.c_str(),
        SERVICE_ALL_ACCESS,
        SERVICE_WIN32_OWN_PROCESS,
        SERVICE_AUTO_START,
        SERVICE_ERROR_NORMAL,
        appPath.toStdWString().c_str(),
        nullptr, nullptr, nullptr, nullptr, nullptr);

    if (!service) {
        qCritical() << "Failed to create service";
        CloseServiceHandle(scm);
        return false;
    }

    // 복구 옵션 설정 (장애 시 자동 재시작)
    SERVICE_FAILURE_ACTIONS failureActions;
    SC_ACTION actions[3];

    // 1차 실패: 재시작 (60초 대기)
    actions[0].Type = SC_ACTION_RESTART;
    actions[0].Delay = 60000; // 60 seconds
    // 2차 실패: 재시작 (60초 대기)
    actions[1].Type = SC_ACTION_RESTART;
    actions[1].Delay = 60000;
    // 3차 이후: 아무 작업 안 함 (무한 루프 방지)
    actions[2].Type = SC_ACTION_NONE;
    actions[2].Delay = 0;

    failureActions.dwResetPeriod = 86400; // 24시간 후 실패 횟수 초기화
    failureActions.lpRebootMsg = nullptr;
    failureActions.lpCommand = nullptr;
    failureActions.cActions = 3;
    failureActions.lpsaActions = actions;

    if (!ChangeServiceConfig2(service, SERVICE_CONFIG_FAILURE_ACTIONS, &failureActions)) {
        qWarning() << "Failed to set service recovery options. (Error:" << GetLastError() << ")";
    } else {
        qDebug() << "Service recovery options configured successfully.";
    }

    qDebug() << "Service installed successfully.";
    CloseServiceHandle(service);
    CloseServiceHandle(scm);
    return true;
}

/**
 * @brief 서비스를 Windows 시스템에서 제거
 */
bool WindowsService::uninstall()
{
    SC_HANDLE scm = OpenSCManager(nullptr, nullptr, SC_MANAGER_ALL_ACCESS);
    if (!scm) {
        qCritical() << "Failed to open Service Control Manager.";
        return false;
    }

    SC_HANDLE service = OpenServiceW(scm, m_serviceName.toStdWString().c_str(), SERVICE_STOP | DELETE);
    if (!service) {
        CloseServiceHandle(scm);
        return false;
    }

    SERVICE_STATUS status;
    ControlService(service, SERVICE_CONTROL_STOP, &status);

    if (DeleteService(service)) {
        qDebug() << "Service uninstalled successfully.";
    } else {
        qCritical() << "Failed to delete service.";
    }

    CloseServiceHandle(service);
    CloseServiceHandle(scm);
    return true;
}

/**
 * @brief 서비스를 시작하고 제어 관리자와 통신 시작
 */
bool WindowsService::run()
{
    std::wstring         serviceName     = m_serviceName.toStdWString();
    SERVICE_TABLE_ENTRYW dispatchTable[] = {
        { const_cast<LPWSTR>(serviceName.c_str()), serviceMain },
        {                                 nullptr,     nullptr }
    };

    return StartServiceCtrlDispatcherW(dispatchTable);
}

void WINAPI WindowsService::serviceMain(DWORD /*argc*/, LPTSTR* /*argv*/)
{
    m_statusHandle = RegisterServiceCtrlHandlerW(
        m_serviceName.toStdWString().c_str(),
        serviceCtrlHandler);

    if (!m_statusHandle) return;

    m_serviceStatus.dwServiceType             = SERVICE_WIN32_OWN_PROCESS;
    m_serviceStatus.dwServiceSpecificExitCode = 0;

    // 서비스가 시작 중임을 보고
    setServiceStatus(SERVICE_START_PENDING);

    int        argc   = 1;
    QByteArray nameBa = m_serviceName.toLocal8Bit();
    char*      argv[] = { nameBa.data(), nullptr };

    // Qt 이벤트 루프를 위한 Application 객체 생성
    QCoreApplication a(argc, argv);

    setServiceStatus(SERVICE_RUNNING);

    // 공통 코어 로직 실행 (블록됨)
    runDaemonCore(a);

    setServiceStatus(SERVICE_STOP_PENDING);
    setServiceStatus(SERVICE_STOPPED);
}

int WindowsService::runDaemonCore(QCoreApplication& a)
{
    // 1. Initialize logging system (Async + Rotating)
    Logger::init();

    // 2. Load JSON configuration (create template if not found)
    const QString cfgPath = ConfigLoader::defaultConfigPath();
    if (!ConfigLoader::load(cfgPath)) {
        ConfigLoader::save(cfgPath);
    }

#ifndef QT_DEBUG
    // install custom message handler for logging
    qInstallMessageHandler(Logger::qtMessageHandler);
#endif

    qDebug() << "---------- Servo Daemon Started ----------";

    // EtherCAT 서버 인스턴스 생성 및 시작
    EcatServer* server = new EcatServer(&a);
    if (server) {
        server->start();
    }

    // Qt 이벤트 루프 시작 (여기서 블록됨)
    int ret = a.exec();

    // Gracefully stop the EtherCAT server before stopping the service
    if (server) {
        server->stop();
    }

    return ret;
}

/**
 * @brief 서비스 상태 변경 요청 처리 (중지, 셧다운 등)
 */
void WINAPI WindowsService::serviceCtrlHandler(DWORD ctrlCode)
{
    switch (ctrlCode) {
    case SERVICE_CONTROL_STOP:
    case SERVICE_CONTROL_SHUTDOWN:
        setServiceStatus(SERVICE_STOP_PENDING);
        if (QCoreApplication::instance()) {
            QCoreApplication::quit();
        }
        break;
    default:
        break;
    }
}

/**
 * @brief SCM에 현재 서비스의 상태를 보고
 */
void WindowsService::setServiceStatus(DWORD currentState, DWORD win32ExitCode, DWORD waitHint)
{
    static DWORD checkPoint         = 1;
    m_serviceStatus.dwCurrentState  = currentState;
    m_serviceStatus.dwWin32ExitCode = win32ExitCode;
    m_serviceStatus.dwWaitHint      = waitHint;

    if (currentState == SERVICE_START_PENDING) {
        m_serviceStatus.dwControlsAccepted = 0;
    } else {
        m_serviceStatus.dwControlsAccepted = SERVICE_ACCEPT_STOP | SERVICE_ACCEPT_SHUTDOWN;
    }

    if (currentState == SERVICE_RUNNING || currentState == SERVICE_STOPPED) {
        m_serviceStatus.dwCheckPoint = 0;
    } else {
        m_serviceStatus.dwCheckPoint = checkPoint++;
    }

    SetServiceStatus(m_statusHandle, &m_serviceStatus);
}
