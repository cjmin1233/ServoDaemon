#ifndef CONFIGLOADER_H
#define CONFIGLOADER_H

#include <QString>

/**
 * @brief 런타임 설정 로더
 *
 * JSON 파일을 파싱하여 ServoConfig::SlaveConfigs 배열과
 * 네트워크 설정(HOST, PORT)을 덮어씁니다.
 *
 * 사용 예시 (main.cpp):
 * @code
 *   ConfigLoader::load(ConfigLoader::defaultConfigPath());
 * @endcode
 *
 * JSON 파일이 없거나 파싱에 실패하면 servoconfig.cpp에 정의된
 * 하드코딩 기본값을 그대로 사용합니다 (폴백).
 */
class ConfigLoader
{
public:
    ConfigLoader() = delete;

    /**
     * @brief JSON 파일을 읽어 전체 설정을 덮어씁니다.
     * @param path JSON 파일 경로
     * @return 성공 시 true, 파일이 없거나 파싱 실패 시 false (기본값 유지)
     */
    static bool load(const QString& path);

    /**
     * @brief 현재 설정값을 JSON 파일로 저장합니다.
     * @param path 저장할 JSON 파일 경로
     * @return 성공 시 true
     */
    static bool save(const QString& path);

    /**
     * @brief 실행 파일 옆 config/servoconfig.json 경로를 반환합니다.
     */
    static QString defaultConfigPath();

    // 네트워크 설정 접근자
    static quint16  port();
    static QString  host();

private:
    static quint16 s_port;
    static QString s_host;
};

#endif // CONFIGLOADER_H
