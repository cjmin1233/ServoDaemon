#ifndef COMMONCONFIG_H
#define COMMONCONFIG_H

#include <QString>
#include <QDataStream>
#include <QList>

enum class MessageType : quint32
{
    Command,            // Client -> Server
    CommandResponse,    // Server -> Client
    ServoStatusUpdate,  // Server -> Client
    SystemInitData,     // Server -> Client
    ServoArriveAlarm,   // Server -> Client
};

enum class CommandType : quint32
{
    MovePosition,
    SetHome,
    SetTorque,
    SetVelocity,
    Pause,
    StopServo,
    Heartbeat,
    MovePoint,
};
enum class ResponseStatus : quint32
{
    ACK,
    NACK
};
enum class ErrorReason : quint32
{
    None = 0,
    InvalidSlaveId,
    ServoFault,
    MasterOffline
};

#pragma pack(push, 1)
struct Command
{
    quint16 slaveId;
    CommandType cmdType;
    qint32 value = 0;
    friend QDataStream& operator<<(QDataStream& out, const Command& cmd)
    {
        out << (quint16)cmd.slaveId;
        out << (quint32)cmd.cmdType;
        out << (qint32)cmd.value;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in, Command& cmd)
    {
        quint32 tempCmdType;
        in >> cmd.slaveId;
        in >> tempCmdType;
        cmd.cmdType = static_cast<CommandType>(tempCmdType);
        in >> cmd.value;
        return in;
    }
};

struct CommandResponse
{
    quint16 slaveId;
    CommandType cmdType;
    ResponseStatus status;
    ErrorReason reason = ErrorReason::None;
    friend QDataStream& operator<<(QDataStream& out, const CommandResponse& resp)
    {
        out << (quint16)resp.slaveId;
        out << (quint32)resp.cmdType;
        out << (quint32)resp.status;
        out << (quint32)resp.reason;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in, CommandResponse& resp)
    {
        quint32 tempCmdType, tempStatus, tempReason;
        in >> resp.slaveId >> tempCmdType >> tempStatus >> tempReason;
        resp.cmdType = static_cast<CommandType>(tempCmdType);
        resp.status = static_cast<ResponseStatus>(tempStatus);
        resp.reason = static_cast<ErrorReason>(tempReason);
        return in;
    }
};

struct ServoStatus
{
    int32_t position = 0;
    int32_t velocity = 0;
    bool hasError = false;
    // bool hasArrived = false;
    uint16_t errorCode = 0;
    friend QDataStream& operator<<(QDataStream& out, const ServoStatus& st)
    {
        out << (qint32)st.position;
        out << (qint32)st.velocity;
        out << (bool)st.hasError;
        // out << (bool)st.hasArrived;
        out << (quint16)st.errorCode;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in, ServoStatus& st)
    {
        in >> st.position >> st.velocity >> st.hasError /*>> st.hasArrived*/ >> st.errorCode;
        return in;
    }
};

struct ServoInitData
{
    quint16 slaveId;
    qint32 minPosition;
    qint32 maxPosition;

    friend QDataStream& operator<<(QDataStream& out, const ServoInitData& data) {
        out << (quint16)data.slaveId;
        out << (qint32)data.minPosition;
        out << (qint32)data.maxPosition;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in, ServoInitData& data) {
        in >> data.slaveId >> data.minPosition >> data.maxPosition;
        return in;
    }
};

struct SystemInitData
{
    quint32 totalServos;
    QList<ServoInitData> servos;
    friend QDataStream& operator<<(QDataStream& out, const SystemInitData& data) {
        out << (quint32)data.totalServos;
        out << data.servos;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in, SystemInitData& data) {
        in >> data.totalServos >> data.servos;
        return in;
    }
};

#pragma pack(pop)

namespace Config {
// 기본값. ConfigLoader::load() 호출 시 JSON 값으로 덮어씌워진다.
inline quint16 PORT = 5000;        // port number
inline QString HOST = "127.0.0.1"; // local loopback address
}

#endif
