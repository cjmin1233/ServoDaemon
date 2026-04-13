#include "windowsservice.h"
#include "ecatserver.h"

#include <QCoreApplication>
#include <QDebug>

QString               WindowsService::m_serviceName   = "";
SERVICE_STATUS_HANDLE WindowsService::m_statusHandle  = nullptr;
SERVICE_STATUS        WindowsService::m_serviceStatus = { 0 };

WindowsService::WindowsService(const QString& serviceName)
{
    m_serviceName = serviceName;
}

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

bool WindowsService::run()
{
    std::wstring         serviceName     = m_serviceName.toStdWString();
    SERVICE_TABLE_ENTRYW dispatchTable[] = {
        { const_cast<LPWSTR>(serviceName.c_str()), serviceMain },
        {                                 nullptr,     nullptr }
    };

    return StartServiceCtrlDispatcherW(dispatchTable);
}

void WindowsService::serviceMain(DWORD /*argc*/, LPTSTR* /*argv*/)
{
    m_statusHandle = RegisterServiceCtrlHandlerW(
        m_serviceName.toStdWString().c_str(),
        serviceCtrlHandler);

    if (!m_statusHandle) return;

    m_serviceStatus.dwServiceType             = SERVICE_WIN32_OWN_PROCESS;
    m_serviceStatus.dwServiceSpecificExitCode = 0;

    setServiceStatus(SERVICE_START_PENDING);

    int              argc   = 1;
    char             arg0[] = "ServoServiceDemo";
    char*            argv[] = { arg0, nullptr };
    QCoreApplication a(argc, argv);

    setServiceStatus(SERVICE_RUNNING);

    qDebug() << "---------- Servo Daemon Started ----------";

    EcatServer* server = new EcatServer(&a);
    server->start();

    a.exec();

    setServiceStatus(SERVICE_STOP_PENDING);
    setServiceStatus(SERVICE_STOPPED);
}

void WindowsService::serviceCtrlHandler(DWORD ctrlCode)
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
