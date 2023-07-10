#include <motor_controller/green_motor_control.h>

using namespace std;
using namespace hr4c::devices;


GreenMotorControl::GreenMotorControl(const YAML::Node& joint_config) {
    motor_ = make_shared<UDDIRTControl>(joint_config);
    motor_->openDevice();
}

void GreenMotorControl::setControlMode(vector<int>& c_modes,
                                       vector<MotorState>& mstate_array) {
    vector<MotorParams> mparams_v;
    getAllMotorParams_(mparams_v);
    motor_->setControlMode(c_modes, mparams_v, mstate_array);
}

void GreenMotorControl::servoOn(int joint_no, MotorState& mstate) {
    MotorParams mparams;
    copyMotorParams(joint_no, mparams);
    motor_->servoOn(joint_no, mparams, mstate);
}

void GreenMotorControl::servoOnAll(vector<MotorState>& mstate_array) {
    vector<MotorParams> mparams_v;
    getAllMotorParams_(mparams_v);
    motor_->servoOnAll(mparams_v, mstate_array);
}

void GreenMotorControl::servoOff(int joint_no, MotorState& mstate) {
    MotorParams mparams;
    copyMotorParams(joint_no, mparams);
    motor_->servoOff(joint_no, mparams, mstate);
}

void GreenMotorControl::servoOffAll(vector<MotorState>& mstate_array) {
    vector<MotorParams> mparams_v;
    getAllMotorParams_(mparams_v);
    motor_->servoOffAll(mparams_v, mstate_array);
}

void GreenMotorControl::sendTargetsAndUpdateStates(vector<double>& targets,
                                                   const shared_ptr<vector<MotorState>>& mstate_array,
                                                   double tick_s,
                                                   bool dummy_flag) {
    vector<MotorParams> mparams_v;
    getAllMotorParams_(mparams_v);
    motor_->sendTargetsAndUpdateStates(targets, mparams_v, *mstate_array, tick_s, dummy_flag);
}

unsigned int GreenMotorControl::getMotorStatus(int joint_no, vector<MotorState>& mstate_array) {
    unsigned int status = 0;
    status |= (mstate_array.at(joint_no).finished & 0x01) << 5;
    status |= (mstate_array.at(joint_no).servo_state & 0x01) << 4;
    status |= (mstate_array.at(joint_no).alarm & 0x01) << 3;
    status |= (mstate_array.at(joint_no).limit_alarm & 0x01) << 2;
    status |= (mstate_array.at(joint_no).device_error & 0x01) << 1;
    status |= (mstate_array.at(joint_no).connection_error & 0x01);

    return status;
}

void GreenMotorControl::alarmReset(int joint_no, MotorState& mstate) {
    MotorParams mparams;
    copyMotorParams(joint_no, mparams);
    motor_->alarmReset(joint_no, mparams, mstate);
}

void GreenMotorControl::shutdown() const {
    // do nothing
}

void GreenMotorControl::calibrateJoint(int joint_no, double calibrate_angle, MotorState& mstate) {
    MotorParams mparam;
    copyMotorParams(joint_no, mparam);
    auto offset = motor_->calibrateJoint(calibrate_angle, mparam, mstate);
    setJointOffset(joint_no, offset);
}

void GreenMotorControl::calibrateJointFromMemory(int joint_no,
                                                 double calibrate_angle,
                                                 double memory_angle,
                                                 MotorState &mstate) {
    MotorParams mparam;
    copyMotorParams(joint_no, mparam);
    auto offset = motor_->calibrateJointFromMemory(calibrate_angle, memory_angle, mparam, mstate);
    setJointOffset(joint_no, offset);
}

void GreenMotorControl::getAllMotorParams_(vector<MotorParams> &mparams_v) {
    for (auto i = 0; i < motor_->getJointNum(); ++i) {
        MotorParams mparams;
        copyMotorParams(i, mparams);
        mparams_v.emplace_back(mparams);
    }
}

void GreenMotorControl::setZeroGMode(bool flag) {
    // TODO: zerogモードについて内部実行するように移植を行う
    // 現状は型を合わせるだけのダミー実装
}
