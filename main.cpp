#include "windowsservice.h"
#include <QCoreApplication>
#include <QString>
#include <iostream>
#include <shlobj.h>
#include <windows.h>

/**
 * @brief Helper function to check if the current process is running with administrator privileges
 * @return true if running with admin privileges, false otherwise
 */
bool isUserAdmin()
{
    return IsUserAnAdmin();
}

int main(int argc, char* argv[])
{
    QString serviceName = "ServoServiceDemo";

    // 1. Check for command line arguments mapping to modes
    bool isConsoleMode   = false;
    bool isInstallMode   = false;
    bool isUninstallMode = false;

    for (int i = 1; i < argc; ++i) {
        QString arg = QString::fromLocal8Bit(argv[i]);
        if (arg == "-c" || arg == "--console") {
            isConsoleMode = true;
        } else if (arg == "-install" || arg == "--install") {
            isInstallMode = true;
            if (i + 1 < argc && !QString::fromLocal8Bit(argv[i + 1]).startsWith("-")) {
                serviceName = QString::fromLocal8Bit(argv[++i]);
            }
        } else if (arg == "-uninstall" || arg == "--uninstall") {
            isUninstallMode = true;
            if (i + 1 < argc && !QString::fromLocal8Bit(argv[i + 1]).startsWith("-")) {
                serviceName = QString::fromLocal8Bit(argv[++i]);
            }
        } else if (arg == "--service") {
            if (i + 1 < argc) {
                serviceName = QString::fromLocal8Bit(argv[++i]);
            }
        }
    }

    // 2. Execute Console/Install Mode (Needs local QCoreApplication context)
    // - In console mode, we run the daemon synchronously.
    // - In install mode, we need QCoreApplication to resolve the absolute executable path using Qt functions.
    if (isConsoleMode || isInstallMode || isUninstallMode) {
        QCoreApplication a(argc, argv);

        WindowsService service(serviceName);

        if (isInstallMode || isUninstallMode) {
            // Service install/uninstall requires administrator privileges
            if (!isUserAdmin()) {
                std::cout << "Error: Administrator privileges are required to install or remove the service.\n";
                std::cout << "Please run this program as an administrator.\n";
                return 1;
            }

            if (isInstallMode) {
                return service.install() ? 0 : 1;
            } else if (isUninstallMode) {
                return service.uninstall() ? 0 : 1;
            }
        }

        if (isConsoleMode) {
            std::cout << "Running in standalone console mode for debugging..." << std::endl;
            return WindowsService::runDaemonCore(a);
        }
    }

    // 3. Execute Service Mode
    // DO NOT instantiate QCoreApplication here! serviceMain() on the SCM thread will create its own.
    WindowsService service(serviceName);

    // Attempt to connect to SCM by calling StartServiceCtrlDispatcher
    if (!service.run()) {
        std::cout << "This application is a Windows service and must be run via the Service Control Manager (SCM).\n";
        std::cout << "To install the service, run: " << argv[0] << " --install [ServiceName]\n";
        std::cout << "To uninstall the service, run: " << argv[0] << " --uninstall [ServiceName]\n";
        std::cout << "To run as a console application for debugging, run: " << argv[0] << " --console\n";
        return 1;
    }

    return 0;
}
