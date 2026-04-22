#include "configloader.h"
#include "../CommonConfig.h"
#include "servoconfig.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

// 정적 멤버 초기값 (CommonConfig.h 기본값과 동일)
quint16 ConfigLoader::s_port = 5000;
QString ConfigLoader::s_host = QStringLiteral("127.0.0.1");

// ─────────────────────────────────────────────
// 퍼블릭 접근자
// ─────────────────────────────────────────────

quint16 ConfigLoader::port() { return s_port; }
QString ConfigLoader::host() { return s_host; }

QString ConfigLoader::defaultConfigPath()
{
    // 실행 파일 옆의 config/servoconfig.json
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/servoconfig.json"));
}

// ─────────────────────────────────────────────
// JSON → ServoParams 파싱 헬퍼
// ─────────────────────────────────────────────

static ServoConfig::ServoParams parseServoParams(const QJsonObject&              obj,
                                                 const ServoConfig::ServoParams& defaults)
{
    auto u32 = [&](const char* key, uint32_t def) -> uint32_t {
        return obj.contains(key) ? static_cast<uint32_t>(obj[key].toDouble(def)) : def;
    };
    auto u16 = [&](const char* key, uint16_t def) -> uint16_t {
        return obj.contains(key) ? static_cast<uint16_t>(obj[key].toDouble(def)) : def;
    };
    auto i32 = [&](const char* key, int32_t def) -> int32_t {
        return obj.contains(key) ? static_cast<int32_t>(obj[key].toDouble(def)) : def;
    };
    auto i16 = [&](const char* key, int16_t def) -> int16_t {
        return obj.contains(key) ? static_cast<int16_t>(obj[key].toDouble(def)) : def;
    };
    auto i8 = [&](const char* key, int8_t def) -> int8_t {
        return obj.contains(key) ? static_cast<int8_t>(obj[key].toDouble(def)) : def;
    };

    ServoConfig::ServoParams p;

    // --- [1] Profile Position Mode ---
    p.profileVelocity     = u32("profileVelocity", defaults.profileVelocity);
    p.profileAccel        = u32("profileAccel", defaults.profileAccel);
    p.profileDecel        = u32("profileDecel", defaults.profileDecel);
    p.stopDecel           = u32("stopDecel", defaults.stopDecel);
    p.posCommandFilter    = u16("posCommandFilter", defaults.posCommandFilter);
    p.posCommandAvgFilter = u16("posCommandAvgFilter", defaults.posCommandAvgFilter);
    p.posLimitFunc        = u16("posLimitFunc", defaults.posLimitFunc);

    // --- [2] Homing Mode ---
    p.homeOffset      = i32("homeOffset", defaults.homeOffset);
    p.homingMethod    = i8("homingMethod", defaults.homingMethod);
    p.homingSpdSwitch = u32("homingSpdSwitch", defaults.homingSpdSwitch);
    p.homingSpdZero   = u32("homingSpdZero", defaults.homingSpdZero);
    p.homingAccel     = u32("homingAccel", defaults.homingAccel);

    // --- [3] Profile Torque Mode ---
    p.torqueLimitFunc  = u16("torqueLimitFunc", defaults.torqueLimitFunc);
    p.speedLimitFunc   = u16("speedLimitFunc", defaults.speedLimitFunc);
    p.posTorqueLimit   = u16("posTorqueLimit", defaults.posTorqueLimit);
    p.negTorqueLimit   = u16("negTorqueLimit", defaults.negTorqueLimit);
    p.torqueSpeedLimit = u16("torqueSpeedLimit", defaults.torqueSpeedLimit);
    p.torqueSlope      = u32("torqueSlope", defaults.torqueSlope);
    p.torqueOffset     = i16("torqueOffset", defaults.torqueOffset);

    // --- [4] etc ---
    p.positionWindow  = u32("positionWindow", defaults.positionWindow);
    p.quickStopOption = i16("quickStopOption", defaults.quickStopOption);
    p.shutdownOption  = i16("shutdownOption", defaults.shutdownOption);
    p.haltOption      = i16("haltOption", defaults.haltOption);

    // --- Mechanical Specs ---
    p.encoderPPR        = u32("encoderPPR", defaults.encoderPPR);
    p.rotationDirection = u16("rotationDirection", defaults.rotationDirection);
    p.motorRevolutions  = u32("motorRevolutions", defaults.motorRevolutions);
    p.shaftRevolutions  = u32("shaftRevolutions", defaults.shaftRevolutions);
    p.leadMm            = u32("leadMm", defaults.leadMm);
    p.strokeMm          = u32("strokeMm", defaults.strokeMm);

    return p;
}

// ─────────────────────────────────────────────
// load / save
// ─────────────────────────────────────────────

bool ConfigLoader::load(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "[ConfigLoader::load] Config file not found:" << path
                   << "→ Using compiled-in defaults.";
        return false;
    }

    QJsonParseError err;
    QJsonDocument   doc = QJsonDocument::fromJson(file.readAll(), &err);

    if (err.error != QJsonParseError::NoError) {
        qWarning() << "[ConfigLoader::load] JSON parse error:" << err.errorString()
                   << "→ Using compiled-in defaults.";
        return false;
    }

    QJsonObject root = doc.object();

    // ── 네트워크 설정 ──────────────────────────
    if (root.contains("network")) {
        QJsonObject net = root["network"].toObject();
        s_port          = static_cast<quint16>(net["port"].toDouble(s_port));
        s_host          = net.value("host").toString(s_host);

        // CommonConfig::Config 의 inline 변수에도 반영
        Config::PORT = s_port;
        Config::HOST = s_host;
    }

    // ── 슬레이브 설정 ──────────────────────────
    if (root.contains("slaves")) {
        QJsonArray slaves = root["slaves"].toArray();

        for (int i = 1; i <= ServoConfig::slaveCountMax; ++i) {
            if (i >= slaves.size()) break;

            QJsonValue val = slaves[i];
            if (val.isNull() || !val.isObject()) continue;

            ServoConfig::SlaveConfigs[i] = parseServoParams(val.toObject(), ServoConfig::SlaveConfigs[i]);

            qInfo() << "[ConfigLoader::load] Slave" << i << "config loaded from JSON.";
        }
    }

    qInfo() << "[ConfigLoader::load] Config loaded successfully:" << path;
    return true;
}

bool ConfigLoader::save(const QString& path)
{
    QJsonArray slaves;
    slaves.append(QJsonValue::Null); // Index 0 (Dummy)

    for (int i = 1; i <= ServoConfig::slaveCountMax; ++i) {
        const auto& p = ServoConfig::SlaveConfigs[i];
        QJsonObject obj;

        obj["profileVelocity"]     = static_cast<qint64>(p.profileVelocity);
        obj["profileAccel"]        = static_cast<qint64>(p.profileAccel);
        obj["profileDecel"]        = static_cast<qint64>(p.profileDecel);
        obj["stopDecel"]           = static_cast<qint64>(p.stopDecel);
        obj["posCommandFilter"]    = p.posCommandFilter;
        obj["posCommandAvgFilter"] = p.posCommandAvgFilter;
        obj["posLimitFunc"]        = p.posLimitFunc;

        obj["homeOffset"]      = p.homeOffset;
        obj["homingMethod"]    = p.homingMethod;
        obj["homingSpdSwitch"] = static_cast<qint64>(p.homingSpdSwitch);
        obj["homingSpdZero"]   = static_cast<qint64>(p.homingSpdZero);
        obj["homingAccel"]     = static_cast<qint64>(p.homingAccel);

        obj["torqueLimitFunc"]  = p.torqueLimitFunc;
        obj["speedLimitFunc"]   = p.speedLimitFunc;
        obj["posTorqueLimit"]   = p.posTorqueLimit;
        obj["negTorqueLimit"]   = p.negTorqueLimit;
        obj["torqueSpeedLimit"] = p.torqueSpeedLimit;
        obj["torqueSlope"]      = static_cast<qint64>(p.torqueSlope);
        obj["torqueOffset"]     = p.torqueOffset;

        obj["positionWindow"]  = static_cast<qint64>(p.positionWindow);
        obj["quickStopOption"] = p.quickStopOption;
        obj["shutdownOption"]  = p.shutdownOption;
        obj["haltOption"]      = p.haltOption;

        obj["encoderPPR"]        = static_cast<qint64>(p.encoderPPR);
        obj["rotationDirection"] = p.rotationDirection;
        obj["motorRevolutions"]  = static_cast<qint64>(p.motorRevolutions);
        obj["shaftRevolutions"]  = static_cast<qint64>(p.shaftRevolutions);
        obj["leadMm"]            = static_cast<qint64>(p.leadMm);
        obj["strokeMm"]          = static_cast<qint64>(p.strokeMm);

        slaves.append(obj);
    }

    QJsonObject network;
    network["host"] = s_host;
    network["port"] = s_port;

    QJsonObject root;
    root["network"] = network;
    root["slaves"]  = slaves;

    // 디렉터리 없으면 생성
    QFileInfo fi(path);
    QDir().mkpath(fi.absolutePath());

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "[ConfigLoader::save] Cannot write config file:" << path;
        return false;
    }

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    qInfo() << "[ConfigLoader::save] Config saved:" << path;
    return true;
}
