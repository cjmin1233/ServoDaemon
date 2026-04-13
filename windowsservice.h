#ifndef WINDOWSSERVICE_H
#define WINDOWSSERVICE_H

#include <QString>
#include <windows.h>

class WindowsService {
public:
    explicit WindowsService(const QString& serviceName);
    ~WindowsService() = default;

    bool install();
    bool uninstall();
    bool run();

private:
    static void WINAPI serviceMain(DWORD argc, LPTSTR* argv);
    static void WINAPI serviceCtrlHandler(DWORD ctrlCode);
    static void        setServiceStatus(DWORD currentState, DWORD win32ExitCode = NO_ERROR, DWORD waitHint = 0);

    static QString               m_serviceName;
    static SERVICE_STATUS_HANDLE m_statusHandle;
    static SERVICE_STATUS        m_serviceStatus;
};

#endif // WINDOWSSERVICE_H
