#ifndef HR4C_COMM_COMMAND_SERVICE_H
#define HR4C_COMM_COMMAND_SERVICE_H
#include <string>
#include <vector>

namespace hr4c_comm::command_service {
    using namespace std;
    constexpr auto CommandSetJointReference = "SetJointReference";
    constexpr auto CommandSetJointTrajectory = "SetJointTrajectory";
    constexpr auto CommandGetJointAngle = "GetJointAngle";
    constexpr auto CommandWaitInterpolation = "WaitInterpolation";
    constexpr auto CommandSetControlMode = "SetControlMode";
    constexpr auto CommandStartLogging = "StartLogging";
    constexpr auto CommandStopLogging = "StopLogging";
    constexpr auto CommandClearLogs = "ClearLogs";
    constexpr auto CommandGetLogList = "GetLogList";
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

    class CommandService {
    public:
        virtual void createCommandString(const string& command, vector<double>& args, string& output_str) = 0;
        virtual void parseCommandResponse(const string& input_str,
                                          string& command, string& response, vector<double>& result,
                                          vector<string> &result_str) = 0;
    };

    class JSONCommandService : public CommandService {
    public:
        JSONCommandService() = default;
        ~JSONCommandService() = default;
        void createCommandString(const string& command, vector<double>& args, string& output_str) override;
        void parseCommandResponse(const string& input_str,
                                  string& command, string& response, vector<double>& result,
                                  vector<string> &result_str) override;
    };
} // namespace hr4c_comm

#endif //HR4C_COMM_COMMAND_SERVICE_H
