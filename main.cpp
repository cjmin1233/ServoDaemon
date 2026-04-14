#include "Logger.h"
#include "windowsservice.h"
#include <QCoreApplication>
#include <iostream>
#include <shlobj.h>
#include <windows.h>

/**
 * @brief 현재 프로세스가 관리자 권한으로 실행 중인지 확인하는 헬퍼 함수
 * @return 관리자 권한이면 true, 아니면 false
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

    // 서비스의 기본 이름 정의
    QString        serviceName = "ServoServiceDemo";
    WindowsService service(serviceName);

    // 명령줄 인자가 있는 경우 (설치/제거 처리)
    if (argc > 1) {
        QString arg(argv[1]);
        if (arg == "-install" || arg == "--install" || arg == "-uninstall" || arg == "--uninstall") {

            // 서비스 설치/제거는 반드시 관리자 권한이 필요함
            if (!isUserAdmin()) {
                std::cout << "오류: 서비스 설치 또는 제거를 위해서는 관리자 권한이 필요합니다.\n";
                std::cout << "이 프로그램을 '관리자 권한으로 실행'해 주세요.\n";
                return 1;
            }

            if (arg == "-install" || arg == "--install") {
                if (service.install()) {
                    std::cout << "서비스가 성공적으로 설치되었습니다.\n";
                    return 0;
                } else {
                    std::cerr << "서비스 설치에 실패했습니다. 이미 설치되어 있는지 확인하십시오.\n";
                    return 1;
                }
            } else if (arg == "-uninstall" || arg == "--uninstall") {
                if (service.uninstall()) {
                    std::cout << "서비스가 성공적으로 제거되었습니다.\n";
                    return 0;
                } else {
                    std::cerr << "서비스 제거에 실패했습니다. 서비스가 존재하지 않거나 권한 문제일 수 있습니다.\n";
                    return 1;
                }
            }
        }
    }

    // 인자가 없거나 서비스로 실행되어야 하는 경우
    // StartServiceCtrlDispatcher를 호출하여 SCM과 연결 시도
    if (!service.run()) {
        std::cout << "이 응용 프로그램은 Windows 서비스이며 서비스 제어 관리자(SCM)를 통해 실행되어야 합니다.\n";
        std::cout << "서비스를 설치하려면 다음을 실행하십시오: " << argv[0] << " -install\n";
        return 1;
    }

    return 0;
}
