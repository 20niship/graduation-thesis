#ifndef HR4C_CORE_MOTOR_CONTROL_H
#define HR4C_CORE_MOTOR_CONTROL_H
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "motor_controller/motor_state.h"
#include "net/udp_client.h"


namespace hr4c::devices {
    using namespace std;
    constexpr auto JOINT_CALIB_FILE = "joint_calib.yaml";

    // パラメータクラス
    class MotorParams {
     public:
        int b_id;
        double gear_ratio;
        double gear_efficiency;
        double torque_constant;
        double max_angle;
        double min_angle;
        double joint_offset;
        int motor_direction;
        unsigned long pulse_per_round;
        string dev_name;
        bool zerog_mode;
    };

    // インタフェースクラス
    class MotorControl {
    public:
        MotorControl() = default;
        ~MotorControl() = default;
        virtual void setControlMode(vector<int>& c_modes,
                                    vector<MotorState>& mstate_vector) = 0;
        virtual void servoOn(int joint_no, MotorState& mstate) = 0;
        virtual void servoOnAll(vector<MotorState>& mstate_vector) = 0;
        virtual void servoOff(int joint_no, MotorState& mstate) = 0;
        virtual void servoOffAll(vector<MotorState>& mstate_vector) = 0;
        virtual void sendTargetsAndUpdateStates(vector<double>& targets,
                                                const shared_ptr<vector<MotorState>>& mstate_vector,
                                                double tick_s,
                                                bool dumy_flag) = 0;
        virtual unsigned int getMotorStatus(int joint_no, vector<MotorState>& mstate_vector) = 0;
        virtual void alarmReset(int joint_no, MotorState& mstate) = 0;
        virtual void shutdown() const = 0;
        virtual void calibrateJoint(int joint_no, double calibrate_angle, MotorState& mstate) = 0;
        virtual void calibrateJointFromMemory(int joint_no, double calibrate_angle, double memory_angle, MotorState& mstate) = 0;
        virtual void setZeroGMode(bool flag) = 0;
        void setMotorParams(YAML::Node joint_config);
        int getJointNum() { return motor_params_.size(); }
        void copyMotorParams(int joint_no, MotorParams& mparam) {
            mparam.b_id = motor_params_.at(joint_no).b_id;
            mparam.gear_ratio = motor_params_.at(joint_no).gear_ratio;
            mparam.gear_efficiency = motor_params_.at(joint_no).gear_efficiency;
            mparam.torque_constant = motor_params_.at(joint_no).torque_constant;
            mparam.max_angle = motor_params_.at(joint_no).max_angle;
            mparam.min_angle = motor_params_.at(joint_no).min_angle;
            mparam.joint_offset = motor_params_.at(joint_no).joint_offset;
            mparam.motor_direction = motor_params_.at(joint_no).motor_direction;
            mparam.pulse_per_round = motor_params_.at(joint_no).pulse_per_round;
            mparam.dev_name = motor_params_.at(joint_no).dev_name;
            mparam.zerog_mode = motor_params_.at(joint_no).zerog_mode;
        }
        void setJointOffset(int joint_no, double offset) {
            motor_params_.at(joint_no).joint_offset = offset;
        }
        void setZeroGModeFlag(int joint_no, bool flag) {
            motor_params_.at(joint_no).zerog_mode = flag;
        }
     private:
        vector<MotorParams> motor_params_;
    };

    // utility functions
    int convertAngle2Pulse(unsigned long pulse_per_round, double gear_ratio, double angle, double offset, int motor_dir);
    double convertPulse2Angle(unsigned long pulse_per_round, double gear_ratio, int pulse, double offset, int motor_dir);
    double convertCurrentDirection(double current, int motor_dir);
    double convertTorque2Current(double torque_constant, double gear_efficiency, double gear_ratio, double torque, int motor_dir);
    double convertCurrent2Torque(double torque_constant, double gear_efficiency, double gear_ratio, double current, int motor_dir);
    double convertRadPerSec2RPM(double gear_ratio, double rad_per_sec, int motor_dir);
    double minMaxCheck(double target, double joint_angle, double min_value, double max_value, int control_mode, int b_id);
}
#endif //HR4C_CORE_MOTOR_CONTROL_H
