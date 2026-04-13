#include "windowsservice.h"
#include <QCoreApplication>

#include <iostream>

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
}
