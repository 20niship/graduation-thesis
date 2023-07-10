#include "hr4c_comm.h"
#include "hr4c_comm_wrapper.h"
#include <algorithm>
#include <cstring>
#include <iostream>

using namespace hr4c_comm;
using namespace std;


// インスタンス作成番号
vector<int> comm_instance_nos;

// インスタンス格納連想配列
map<int, shared_ptr<HR4CSocketClient> > hr4c_comms;

// 同じIP/ポートの組み合わせが既にあるかどうかをチェック
bool check_ip_port_registered(const string& server_ip, int controller_port) {
    for (auto & ino : comm_instance_nos) {
        if (hr4c_comms[ino]->checkIdentity(server_ip, controller_port)) {
            // どちらも一致する場合は登録済み
            return true;
        }
    }

    // 一つも一致するものがなかったので登録なし
    return false;
}

// インスタンス作成関数
int create_hr4c_comm_instance(string server_ip, int controller_port, int dof, string model) {
    auto newly_created_no = 0;
    if(comm_instance_nos.empty()) {
        newly_created_no = 0;
    } else {
        // 登録されている番号の中で最大のものに＋１したものを返す
        auto max_registered_no = *max_element(comm_instance_nos.begin(),
                                              comm_instance_nos.end());
        newly_created_no = max_registered_no + 1;
    }
    comm_instance_nos.emplace_back(newly_created_no);
    hr4c_comms[newly_created_no] = make_shared<HR4CSocketClient>(std::move(server_ip), controller_port, dof, model);

    return newly_created_no;
}

// インスタンス破壊関数
int destroy_hr4c_comm_instance(int instance_no) {
    try {
        auto itr = hr4c_comms.find(instance_no);
        if( itr != hr4c_comms.end() ) {
            hr4c_comms.erase(itr);
            comm_instance_nos.erase(std::remove(comm_instance_nos.begin(),
                                                comm_instance_nos.end(),
                                                instance_no),
                                    comm_instance_nos.end());
        }
        return 0;
    } catch (...) {
        cerr << "Failed to destroy instance: no. " << instance_no << endl;
        return -1;
    }
}

// インスタンス取得関数
shared_ptr<HR4CSocketClient> get_hr4c_comm_instance(int instance_no) {
    try {
        auto itr = hr4c_comms.find(instance_no);
        if( itr != hr4c_comms.end() ) {
            return hr4c_comms[instance_no];
        } else {
            cerr << "No such instance: no. " << instance_no << endl;
            return nullptr;
        }
    } catch (...) {
        cerr << "Failed to get instance: no. " << instance_no << endl;
        return nullptr;
    }
}

int hr4capi_open(char* ip_addr, int controller_port, int dof, const char* model) {
    string server_ip = ip_addr;
    string robot_model = model;

    // 既に他のインスタンスで登録されている組み合わせの場合ははねる
    if (check_ip_port_registered(server_ip, controller_port)) {
        return -1;
    }

    return create_hr4c_comm_instance(server_ip, controller_port, dof, robot_model);
}

int hr4capi_close(int instance_no) {
    return destroy_hr4c_comm_instance(instance_no);
}

int hr4capi_start(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        return hr4c_instance->start();
    } else {
        return -1;
    }
}

int hr4capi_stop(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        return hr4c_instance->stop();
    } else {
        return -1;
    }
}

void hr4capi_set_joint_reference(int instance_no, double *joint_references, int *masks) {
    // joint_references
    vector<double> ref, maskv;

    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        maskv.assign(masks, masks + dof);
        ref.reserve(dof + dof);
        ref.assign(joint_references, joint_references + dof);
        ref.insert(ref.end(), maskv.begin(), maskv.end());
        hr4c_instance->setJointReference(ref);
    }
}

void hr4capi_set_joint_trajectory(int instance_no,
                                  double *goal_angles,
                                  double goal_time,
                                  int interpolation_method,
                                  int *masks) {
    // joint_trajectory
    vector<double> ref, goalv, maskv;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        ref.reserve(dof + 2 + dof);
        goalv.assign(goal_angles, goal_angles + dof);
        maskv.assign(masks, masks + dof);
        ref.insert(ref.end(), goalv.begin(), goalv.end());
        ref.emplace_back(goal_time);
        ref.emplace_back(static_cast<double>(interpolation_method));
        ref.insert(ref.end(), maskv.begin(), maskv.end());
        hr4c_instance->setJointTrajectory(ref);
    }
}

void hr4capi_get_joint_angle(int instance_no, double *joint_angles) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        res.reserve(dof);
        hr4c_instance->getJointAngle(res);
        if (res.size() == dof) {
            for (auto i = 0; i < dof; ++i) {
                joint_angles[i] = res[i];
            }
        }
    }
}

void hr4capi_wait_interpolation(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->waitInterpolation();
    }
}

void hr4capi_set_control_mode(int instance_no, int *control_modes) {
    vector<double> ref;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        ref.assign(control_modes, control_modes + dof);
        hr4c_instance->setControlMode(ref);
    }
}

void hr4capi_start_logging(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->startLogging();
    }
}

void hr4capi_stop_logging(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->stopLogging();
    }
}

void hr4capi_clear_logs(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->clearLogs();
    }
}

int hr4capi_get_lognum(int instance_no) {
    vector<string> log_paths;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->getLogList(log_paths);

        return log_paths.size();
    } else {
        return -1;
    }
}

void hr4capi_get_loglist(int instance_no, char* ret_cstr) {
    vector<string> log_paths;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->getLogList(log_paths);

        string res_str;
        for (auto &log_path : log_paths) {
            res_str += log_path + ",";
        }
        strcpy(ret_cstr, res_str.c_str());
    }
}

int hr4capi_get_log(int instance_no, const char* filename) {
    string filename_str = filename;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        return hr4c_instance->getLog(filename_str);
    } else {
        return -1;
    }
}

void hr4capi_servo_on(int instance_no, int *joint_nos, int size){
    vector<double> joint_no_vec;
    joint_no_vec.assign(joint_nos, joint_nos + size);
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->servoOn(joint_no_vec);
    }
}

void hr4capi_servo_all_on(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->servoAllOn();
    }
}

void hr4capi_servo_off(int instance_no, int *joint_nos, int size) {
    vector<double> joint_no_vec;
    joint_no_vec.assign(joint_nos, joint_nos + size);
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->servoOff(joint_no_vec);
    }
}

void hr4capi_servo_all_off(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->servoAllOff();
    }
}

void hr4capi_get_control_mode(int instance_no, int* control_mode) {
    vector<double> res;

    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        hr4c_instance->getControlMode(res);
        if (res.size() == dof) {
            for (auto i = 0; i < dof; ++i) {
                control_mode[i] = static_cast<int>(res[i]);
            }
        }
    }
}

void hr4capi_get_joint_current(int instance_no, double* joint_current) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        res.reserve(dof);
        hr4c_instance->getJointCurrent(res);
        if (res.size() == dof) {
            for (auto i = 0; i < dof; ++i) {
                joint_current[i] = res[i];
            }
        }
    }
}

void hr4capi_calibrate_joint(int instance_no, int joint_no, double calibrate_angle) {
    vector<double> ref;
    ref.reserve(2);
    ref.emplace_back(static_cast<double>(joint_no));
    ref.emplace_back(calibrate_angle);
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->calibrateJoint(ref);
    }
}

void hr4capi_calibrate_joint_from_memory(int instance_no, int joint_no, double calibrate_angle, double memory_angle) {
    vector<double> ref;
    ref.reserve(3);
    ref.emplace_back(static_cast<double>(joint_no));
    ref.emplace_back(calibrate_angle);
    ref.emplace_back(memory_angle);
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->calibrateJointFromMemory(ref);
    }
}

void hr4capi_alarm_reset(int instance_no, int *joint_nos, int size){
    vector<double> joint_no_vec;
    joint_no_vec.assign(joint_nos, joint_nos + size);
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->alarmReset(joint_no_vec);
    }
}

void hr4capi_get_motor_status(int instance_no, int* motor_status) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        res.reserve(dof);
        hr4c_instance->getMotorStatus(res);
        if (res.size() == dof) {
            for (auto i = 0; i < dof; ++i) {
                motor_status[i] = static_cast<int>(res[i]);
            }
        }
    }
}

void hr4capi_force_stop(int instance_no, int *methods, int size){
    vector<double> method_vec;
    method_vec.assign(methods, methods + size);
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->forceStop(method_vec);
    }
}

void hr4capi_get_joint_speed(int instance_no, double* joint_speed) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        res.reserve(dof);
        hr4c_instance->getJointSpeed(res);
        if (res.size() == dof) {
            for (auto i = 0; i < dof; ++i) {
                joint_speed[i] = res[i];
            }
        }
    }
}

void hr4capi_get_joint_torque(int instance_no, double* joint_torque) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        res.reserve(dof);
        hr4c_instance->getJointTorque(res);
        if (res.size() == dof) {
            for (auto i = 0; i < dof; ++i) {
                joint_torque[i] = res[i];
            }
        }
    }
}

int hr4capi_start_teaching(int instance_no) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->startTeaching(res);
        return static_cast<int>(res.at(0));
    } else {
        return -1;
    }
}

int hr4capi_stop_teaching(int instance_no) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->stopTeaching(res);
        return static_cast<int>(res.at(0));
    } else {
        return -1;
    }
}

int hr4capi_replay_motion(int instance_no, int replay_id, int* masks) {
    vector<double> ref, maskv;
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        ref.reserve(1 + dof);
        maskv.assign(masks, masks + dof);
        ref.emplace_back(static_cast<double>(replay_id));
        ref.insert(ref.end(), maskv.begin(), maskv.end());
        hr4c_instance->replayMotion(ref, res);

        return static_cast<int>(res.at(0));
    } else {
        return -1;
    }
}

void hr4capi_get_motion_list(int instance_no, char* ret_cstr) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->getMotionList(res);
        string res_str;
        for (double rs : res) {
            res_str += to_string(static_cast<int>(rs)) + ",";
        }
        strcpy(ret_cstr, res_str.c_str());
    }
}

int hr4capi_clear_motion(int instance_no, int clear_id) {
    vector<double> ref;
    vector<double> res;
    ref.emplace_back(static_cast<double>(clear_id));
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->clearMotion(ref, res);
        return static_cast<int>(res.at(0));
    } else {
        return -1;
    }
}

void hr4capi_clear_all_motions(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->clearAllMotions();
    }
}

void hr4capi_controller_shutdown(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->controllerShutdown();
    }
}

int hr4capi_ping(int instance_no) {
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto ret = hr4c_instance->ping();
        if (ret) {
            return 1;
        } else {
            return 0;
        }
    } else {
        return -1;
    }
}

int hr4capi_update_controller(int instance_no, const char* saved_directory) {
    string saved_directory_str = saved_directory;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        return hr4c_instance->updateController(saved_directory_str);
    } else {
        return -1;
    }
}

void hr4capi_get_all_sensor_info(int instance_no, double *joint_angles, double* joint_current, double* joint_speed,
                                 double* joint_torque, int* control_mode, int* motor_status) {
    vector<double> res;
    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        auto dof = hr4c_instance->getDof();
        res.reserve(dof * 6);
        hr4c_instance->getAllSensorInfo(res);
        if (res.size() == dof * 6) {
            for (auto i = 0; i < dof; ++i) {
                joint_angles[i] = res[i];
                joint_current[i] = res[6 + i];
                joint_speed[i] = res[12 + i];
                joint_torque[i] = res[18 + i];
                control_mode[i] = static_cast<int>(res[24 + i]);
                motor_status[i] = static_cast<int>(res[30 + i]);
            }
        }
    }
}

void hr4capi_enable_zerog_mode(int instance_no, int on_off) {
    vector<double> on_off_vec;
    on_off_vec.emplace_back(static_cast<double>(on_off));

    auto hr4c_instance = get_hr4c_comm_instance(instance_no);
    if (hr4c_instance != nullptr) {
        hr4c_instance->enableZeroGMode(on_off_vec);
    }
}