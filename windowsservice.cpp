#include "windowsservice.h"
#include "ecatserver.h"
#include "Logger.h"

#include <QCoreApplication>
#include <QDebug>

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
    appPath = "\"" + appPath + "\"";

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

/**
 * @brief 서비스의 실제 진입점 (Windows SCM에 의해 호출됨)
 */
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

    setServiceStatus(SERVICE_RUNNING);

    // Qt 이벤트 루프 시작 (여기서 블록됨)
    a.exec();

    // Gracefully stop the EtherCAT server before stopping the service
    if (server) {
        server->stop();
    }

    setServiceStatus(SERVICE_STOP_PENDING);
    setServiceStatus(SERVICE_STOPPED);
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
