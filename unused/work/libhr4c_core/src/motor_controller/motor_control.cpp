#include <fstream>
#include <thread>
#include <utility>
#include <spdlog/spdlog.h>
#include "motor_controller/motor_control.h"

using namespace hr4c::devices;
using namespace std;

int hr4c::devices::convertAngle2Pulse(unsigned long pulse_per_round, double gear_ratio, double angle, double offset, int motor_dir) {
    // radian -> pulse
    auto sign = motor_dir / abs(motor_dir);
    return static_cast<int>((double)pulse_per_round * gear_ratio * angle * sign / (2 * M_PI) + offset);
}

double hr4c::devices::convertPulse2Angle(unsigned long pulse_per_round, double gear_ratio, int pulse, double offset, int motor_dir) {
    // pulse -> radian
    auto sign = motor_dir / abs(motor_dir);
    return static_cast<double>(2 * M_PI * (pulse - offset) * sign / ((double)pulse_per_round * gear_ratio));
}

double hr4c::devices::convertCurrentDirection(double current, int motor_dir) {
    auto sign = motor_dir / abs(motor_dir);
    return sign * current;
}

double hr4c::devices::convertTorque2Current(double torque_constant, double gear_efficiency, double gear_ratio, double torque, int motor_dir) {
    auto sign = motor_dir / abs(motor_dir);
    return sign * torque / (torque_constant * gear_efficiency * gear_ratio);
}

double hr4c::devices::convertCurrent2Torque(double torque_constant, double gear_efficiency, double gear_ratio, double current, int motor_dir) {
    auto sign = motor_dir / abs(motor_dir);
    return sign * current * (torque_constant * gear_efficiency * gear_ratio);
}

double hr4c::devices::convertRadPerSec2RPM(double gear_ratio, double rad_per_sec, int motor_dir) {
    auto sign = motor_dir / abs(motor_dir);
    return sign * (rad_per_sec * 60.0) * gear_ratio / (2 * M_PI);
}

double hr4c::devices::minMaxCheck(double target, double joint_angle,
                                  double min_value, double max_value, int control_mode, int b_id) {

    // min/maxチェックを行った上でフィルタリングした値を返す
    switch(control_mode) {
        case ControlModePosition:
            if (target < min_value) {
                spdlog::warn("Target(" + to_string(target) + ") is beyond it's min range at motor " + to_string(b_id));
                return min_value;
            } else if(target > max_value) {
                spdlog::warn("Target(" + to_string(target) + ") is beyond it's max range at motor " + to_string(b_id));
                return max_value;
            }
            break;
        case ControlModeSpeed:
        case ControlModeCurrent:
        case ControlModeTorque:
            if (joint_angle < min_value && target < 0) {
                spdlog::warn("Target(" + to_string(target) + ") is beyond it's min range and output is set 0 at motor "  + to_string(b_id));
                return 0.0;
            } else if(joint_angle > max_value && target > 0) {
                spdlog::warn("Target(" + to_string(target) + ") is beyond it's max range and output is set 0 at motor "  + to_string(b_id));
                return 0.0;
            }
            break;
        default:
            break;
    }

    return target;
}

void MotorControl::setMotorParams(YAML::Node joint_config) {
    // set parameters
    auto joint_names = joint_config["joint_names"].as<vector<string>>();
    spdlog::info("Number of joints: " + to_string(joint_names.size()));
    auto joint_no = 0;
    motor_params_.resize(joint_names.size());
    for (const auto& jn : joint_names) {
        motor_params_.at(joint_no).b_id = joint_config[jn]["b_id"].as<int>();
        motor_params_.at(joint_no).gear_ratio = joint_config[jn]["gear_ratio"].as<double>();
        motor_params_.at(joint_no).gear_efficiency = joint_config[jn]["gear_efficiency"].as<double>();
        motor_params_.at(joint_no).torque_constant = joint_config[jn]["torque_constant"].as<double>();
        motor_params_.at(joint_no).pulse_per_round = joint_config[jn]["pulse_per_round"].as<unsigned long>();
        motor_params_.at(joint_no).max_angle = joint_config[jn]["max"].as<double>();
        motor_params_.at(joint_no).min_angle = joint_config[jn]["min"].as<double>();
        motor_params_.at(joint_no).motor_direction = joint_config[jn]["dir"].as<int>();
        motor_params_.at(joint_no).dev_name = joint_config[jn]["dev"].as<string>();
        motor_params_.at(joint_no).joint_offset = 0.0;
        motor_params_.at(joint_no).zerog_mode = false;
        joint_no++;
    }

    // load calibrated results
    std::ifstream ifs(JOINT_CALIB_FILE);
    if (ifs.is_open()) {
        const YAML::Node calib_result {YAML::LoadFile(JOINT_CALIB_FILE)};
        for (auto i = 0; i < motor_params_.size(); ++i) {
            if (calib_result[i]) {
                motor_params_.at(i).joint_offset = calib_result[i].as<double>();
            }
        }
    }
}
