#include "Logger.h"
#include "windowsservice.h"
#include <QCoreApplication>
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
    QCoreApplication a(argc, argv);

    // Initialize logging system (Async + Rotating)
    Logger::init();

    // Define the default name of the service
    QString        serviceName = "ServoServiceDemo";
    WindowsService service(serviceName);

    // If there are command line arguments (handle install/uninstall)
    if (argc > 1) {
        QString arg(argv[1]);
        if (arg == "-install" || arg == "--install" || arg == "-uninstall" || arg == "--uninstall") {

            // Service install/uninstall requires administrator privileges
            if (!isUserAdmin()) {
                std::cout << "Error: Administrator privileges are required to install or remove the service.\n";
                std::cout << "Please run this program as an administrator.\n";
                return 1;
            }

            if (arg == "-install" || arg == "--install") {
                return service.install() ? 0 : 1;
            } else if (arg == "-uninstall" || arg == "--uninstall") {
                return service.uninstall() ? 0 : 1;
            }
        }
    }

    // If no arguments or should run as a service
    // Attempt to connect to SCM by calling StartServiceCtrlDispatcher
    if (!service.run()) {
        std::cout << "This application is a Windows service and must be run via the Service Control Manager (SCM).\n";
        std::cout << "To install the service, run: " << argv[0] << " -install\n";
        return 1;
    }

    return 0;
}
