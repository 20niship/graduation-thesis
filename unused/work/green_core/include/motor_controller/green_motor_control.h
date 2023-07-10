#ifndef GREEN_CORE_MOTOR_CONTROL_H
#define GREEN_CORE_MOTOR_CONTROL_H
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <motor_controller/motor_state.h>
#include <devices/udd_irt_control.h>


namespace hr4c::devices {
    using namespace std;

    constexpr auto GreenCoreVersion = 1.11;
    class GreenMotorControl : public MotorControl {
     public:
        GreenMotorControl() = default;
        ~GreenMotorControl() = default;
        explicit GreenMotorControl(const YAML::Node& joint_config);
        void setControlMode(vector<int>& c_modes,
                            vector<MotorState>& mstate_array) override;
        void servoOn(int joint_no, MotorState& mstate) override;
        void servoOnAll(vector<MotorState>& mstate_array) override;
        void servoOff(int joint_no, MotorState& mstate) override;
        void servoOffAll(vector<MotorState>& mstate_array) override;
        void sendTargetsAndUpdateStates(vector<double>& targets,
                                        const shared_ptr<vector<MotorState>>& mstate_array,
                                        double tick_s,
                                        bool dummy_flag) override;
        unsigned int getMotorStatus(int joint_no, vector<MotorState>& mstate_array) override;
        void alarmReset(int joint_no, MotorState& mstate) override;
        void shutdown() const override;
        void calibrateJoint(int joint_no, double calibrate_angle, MotorState& mstate) override;
        void calibrateJointFromMemory(int joint_no, double calibrate_angle, double memory_angle, MotorState& mstate) override;
        void setZeroGMode(bool flag) override;
     private:
        shared_ptr<UDDIRTControl> motor_;
        void getAllMotorParams_(vector<MotorParams>& mparams_v);
    };
}
#endif //GREEN_CORE_MOTOR_CONTROL_H
