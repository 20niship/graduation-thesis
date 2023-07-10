#ifndef HR4C_CORE_TELEGRAM_PARSER_H
#define HR4C_CORE_TELEGRAM_PARSER_H
#include <string>
#include <vector>

namespace hr4c::telegram {
    using namespace std;
    constexpr auto CommandParseError = "CommandParseError";
    constexpr auto CommandSetJointReference = "SetJointReference";
    constexpr auto CommandSetJointTrajectory = "SetJointTrajectory";
    constexpr auto CommandGetJointAngle = "GetJointAngle";
    constexpr auto CommandWaitInterpolation = "WaitInterpolation";
    constexpr auto CommandSetControlMode = "SetControlMode";
    constexpr auto CommandServoOn = "ServoOn";
    constexpr auto CommandServoOff = "ServoOff";
    constexpr auto CommandServoAllOn = "ServoAllOn";
    constexpr auto CommandServoAllOff = "ServoAllOff";
    constexpr auto CommandGetControlMode = "GetControlMode";
    constexpr auto CommandGetJointCurrent = "GetJointCurrent";
    constexpr auto CommandCalibrateJoint = "CalibrateJoint";
    constexpr auto CommandCalibrateJointFromMemory = "CalibrateJointFromMemory";
    constexpr auto CommandAlarmReset = "AlarmReset";
    constexpr auto CommandGetMotorStatus = "GetMotorStatus";
    constexpr auto CommandForceStop = "ForceStop";
    constexpr auto CommandGetJointSpeed = "GetJointSpeed";
    constexpr auto CommandGetJointTorque = "GetJointTorque";
    constexpr auto CommandStartLogging = "StartLogging";
    constexpr auto CommandStopLogging = "StopLogging";
    constexpr auto CommandClearLogs = "ClearLogs";
    constexpr auto CommandGetLogList = "GetLogList";
    constexpr auto CommandStartTeaching = "StartTeaching";
    constexpr auto CommandStopTeaching = "StopTeaching";
    constexpr auto CommandReplayMotion = "ReplayMotion";
    constexpr auto CommandGetMotionList = "GetMotionList";
    constexpr auto CommandClearMotion = "ClearMotion";
    constexpr auto CommandClearAllMotions = "ClearAllMotions";
    constexpr auto CommandControllerShutdown = "CommandControllerShutdown";
    constexpr auto CommandGetPONG = "CommandGetPONG";
    constexpr auto CommandGetAllSensorInfo = "CommandGetAllSensorInfo";
    constexpr auto CommandSetZeroGMode = "CommandSetZeroGMode";
    constexpr auto ResponseOK = "Success";
    constexpr auto ResponseNG = "Failed";

    struct Command {
        string command;
        vector<double> args;
    };

    class TelegramParser {
    public:
        virtual void parseTelegrams(const string& input_str, vector<Command>& commands) = 0;

        virtual void createResponseString(const string& command,
                                          string& response, vector<double>& result,
                                          vector<string>& result_str, string& output_str) = 0;
    };

    class JSONTelegramParser : public TelegramParser {
    public:
        JSONTelegramParser() = default;
        ~JSONTelegramParser() = default;
        void parseTelegrams(const string& input_str, vector<Command>& commands) override;
        void createResponseString(const string& command,
                                  string& response, vector<double>& result,
                                  vector<string>& result_str, string& output_str) override;
    };
} // namespace hr4c_core::telegram
#endif //HR4C_CORE_TELEGRAM_PARSER_H
