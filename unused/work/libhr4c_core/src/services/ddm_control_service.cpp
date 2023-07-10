#include <chrono>
#include <thread>
#include <bits/stdc++.h>
#include <spdlog/spdlog.h>
#include "services/ddm_control_service.h"
#include "utils/hr4c_utils.h"

using namespace hr4c::services;
using namespace hr4c::utils;


DDMControlService::DDMControlService(const shared_ptr<MotorControl>& motor_controller) : motor_controller_(motor_controller) {
    first_flag_ = true;
    wait_interpolation_flag_ = false;
}

void DDMControlService::shutdownDDMControllers() {
    motor_controller_->shutdown();
}

void DDMControlService::servoOn(int joint_no, vector<double>& last_targets, MotorState& mstate) {
    spdlog::info("ServoON: Joint" + to_string(joint_no + 1));
    if (joint_no < 0 || joint_no >= motor_controller_->getJointNum()) {
        spdlog::error("Invalid joint_no: " + to_string(joint_no + 1));
        return;
    }
    unique_lock<mutex> lck {mtx_};
    motor_controller_->servoOn(joint_no, mstate);
    // last_targetの更新
    switch (mstate.control_mode) {
        case ControlModePosition:
            last_targets.at(joint_no) = mstate.angle;
            break;
        case ControlModeCurrent:
            last_targets.at(joint_no) = mstate.current;
            break;
        case ControlModeTorque:
            last_targets.at(joint_no) = mstate.torque;
            break;
        default:
            last_targets.at(joint_no) = 0.0;
            break;
    }
    lck.unlock();
}

void DDMControlService::servoAllOn(vector<double>& last_targets, vector<MotorState>& mstate_vector) {
    spdlog::info("ServoON: All");
    unique_lock<mutex> lck {mtx_};
    motor_controller_->servoOnAll(mstate_vector);
    for (auto i = 0; i < motor_controller_->getJointNum(); ++i ) {
        // last_targetの更新
        switch (mstate_vector.at(i).control_mode) {
            case ControlModePosition:
                last_targets.at(i) = mstate_vector.at(i).angle;
                break;
            case ControlModeCurrent:
                last_targets.at(i) = mstate_vector.at(i).current;
                break;
            case ControlModeTorque:
                last_targets.at(i) = mstate_vector.at(i).torque;
                break;
            default:
                last_targets.at(i) = 0.0;
                break;
        }
    }
    lck.unlock();
}

void DDMControlService::servoOff(int joint_no, MotorState& mstate) {
    spdlog::info("ServoOFF: Joint" + to_string(joint_no + 1));
    if (joint_no < 0 || joint_no >= motor_controller_->getJointNum()) {
        spdlog::error("Invalid joint_no: " + to_string(joint_no + 1));
        return;
    }
    unique_lock<mutex> lck {mtx_};
    motor_controller_->servoOff(joint_no, mstate);
    lck.unlock();
}

void DDMControlService::servoAllOff(vector<MotorState>& mstate_vector) {
    spdlog::info("ServoOFF: All");
    unique_lock<mutex> lck {mtx_};
    motor_controller_->servoOffAll(mstate_vector);
    lck.unlock();
}

void DDMControlService::servoControl_(vector<double>& targets,
                                      const shared_ptr<vector<MotorState>>& mstate_vector,
                                      bool dummy_flag) {
    chrono::system_clock::time_point now = chrono::system_clock::now();
    auto elapsed_ns = chrono::duration_cast<std::chrono::nanoseconds>(now - last_updated_).count();
    motor_controller_->sendTargetsAndUpdateStates(targets, mstate_vector, elapsed_ns / 1000000000.0, dummy_flag);
    last_updated_ = chrono::system_clock::now();
}

void DDMControlService::setControlModeWhenReady(vector<int>& c_modes, vector<MotorState>& mstate_vector, vector<double>& last_targets) {
    // デフォルト動作として補間中は受け付けないようにする
    unique_lock<mutex> lck {mtx_};
    if (joint_targets_ != nullptr) {
        spdlog::error("Error: interpolation is not finished");
        lck.unlock();
        return;
    }

    vector<int> prev_controlmode;
    for (auto i = 0; i < motor_controller_->getJointNum(); ++i ) {
        prev_controlmode.push_back(mstate_vector.at(i).control_mode);
    }

    motor_controller_->setControlMode(c_modes, mstate_vector);

    for (auto i = 0; i < motor_controller_->getJointNum(); ++i ) {
        if (c_modes.at(i) != prev_controlmode.at(i)) {
            // last_targetの更新
            switch (c_modes.at(i)) {
            case ControlModePosition:
                last_targets.at(i) = mstate_vector.at(i).angle;
                break;
            case ControlModeCurrent:
                last_targets.at(i) = mstate_vector.at(i).current;
                break;
            case ControlModeTorque:
                last_targets.at(i) = mstate_vector.at(i).torque;
                break;
            default:
                last_targets.at(i) = 0.0;
                break;
            }
        }
    }
    lck.unlock();
}

void DDMControlService::setJointTargetsWhenReady(const shared_ptr<deque<vector<double> > >& targets,
                                                 const shared_ptr<vector<int> >& masks)
{
    unique_lock<mutex> lck {mtx_};
    // ユーザがwait interpolation実行中は上書き出来ない
    if (wait_interpolation_flag_) {
        spdlog::error("Error: wait interpolation command is executing");
        lck.unlock();
        return;
    }

    // wait intepolationしていない補間動作中の場合は、参照しているものを破棄し、上書きできる
    if (joint_targets_ != nullptr) {
        spdlog::info("Joint trajectory was overwritten.");
        joint_targets_ = nullptr;
        masks_ = nullptr;
    }

    // 所有権を移す
    joint_targets_ = move(targets);
    masks_ = move(masks);
    lck.unlock();
}

void DDMControlService::controlDDM(vector<double>& last_targets, shared_ptr<vector<MotorState>>& mstate_vector)
{
    unique_lock<mutex> lck {mtx_};
    if (joint_targets_ != nullptr && !joint_targets_->empty()) {
        vector<double> targets = joint_targets_->front();
        // maskが設定されているところは更新しない
        if (masks_ != nullptr) {
            for (auto i = 0; i < masks_->size(); ++i) {
                if (masks_->at(i) != 0) {
                    targets[i] = last_targets[i];
                }
            }
        }

        // 全軸一気に制御する
        servoControl_(targets, mstate_vector, false);

        // 最後に送ったターゲットの値を更新
        for (int i = 0; i < motor_controller_->getJointNum(); i++) {
            last_targets[i] = targets[i];
        }

        // 先頭を削除
        joint_targets_->pop_front();
        if (joint_targets_->empty()) {
            // 空になったらnullにして、メモリを開放する
            joint_targets_ = nullptr;
            masks_ = nullptr;
        }
    } else {
        dummyControlForUpdates(last_targets, mstate_vector);
    }
    lck.unlock();
}

void DDMControlService::intervalSleep(long tick_ns, chrono::system_clock::time_point start) {
    auto nsec_per_sec = 1000 * 1000 * 1000;
    struct timespec ts{};
    chrono::system_clock::time_point end;
    end = chrono::system_clock::now();
    auto elapsed = chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    auto sleeptime = tick_ns - elapsed;

    if (sleeptime > 0) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_nsec += sleeptime;
        while (ts.tv_nsec >= nsec_per_sec) {
            ts.tv_nsec -= nsec_per_sec;
            ts.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    } else {
        spdlog::warn("could not sleep: " + to_string(sleeptime) + "[ns]");
    }
}

void DDMControlService::waitInterpolation() {
    wait_interpolation_flag_ = true;
    while (true) {
        if (joint_targets_ == nullptr) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    wait_interpolation_flag_ = false;
}

void DDMControlService::calibrateJointWhenReady(int joint_no, double calibrate_angle, MotorState& mstate) {
    // デフォルト動作として補間中は受け付けないようにする
    unique_lock<mutex> lck {mtx_};
    if (joint_targets_ != nullptr) {
        spdlog::error("Error: interpolation is not finished");
        lck.unlock();
        return;
    }

    motor_controller_->calibrateJoint(joint_no, calibrate_angle, mstate);
    writeJointOffsetsToFile();
    lck.unlock();
}

void DDMControlService::calibrateJointFromMemoryWhenReady(int joint_no, double calibrate_angle, double memory_angle, MotorState& mstate) {
    // デフォルト動作として補間中は受け付けないようにする
    unique_lock<mutex> lck {mtx_};
    if (joint_targets_ != nullptr) {
        spdlog::error("Error: interpolation is not finished");
        lck.unlock();
        return;
    }

    motor_controller_->calibrateJointFromMemory(joint_no, calibrate_angle, memory_angle, mstate);
    writeJointOffsetsToFile();
    lck.unlock();
}

void DDMControlService::alarmReset(int joint_no, double current_reference, MotorState& mstate) {
    spdlog::info("AlarmReset: Joint" + to_string(joint_no + 1));
    if (joint_no < 0 || joint_no >= motor_controller_->getJointNum()) {
        spdlog::error("Invalid joint_no: " + to_string(joint_no + 1));
        return;
    }

    unique_lock<mutex> lck {mtx_};
    motor_controller_->alarmReset(joint_no, mstate);
    lck.unlock();
}

void DDMControlService::forceStop(int method, vector<MotorState>& motor_states, double tick_s) {
    unique_lock<mutex> lck {mtx_};

    if (joint_targets_ == nullptr) {
        spdlog::error("No motion is executed!");
        return;
    }

    switch(method) {
        case MethodSuddenStop:
            joint_targets_ = nullptr;
            break;
        case MethodDeceleration:
        default:
            int deceleration_times = static_cast<int>(DecelerationSec / tick_s);
            shared_ptr<deque<vector<double> > > ref_array = make_shared<deque<vector<double> > >();
            vector<vector<double> > decelerated_results{};
            decelerated_results.resize(motor_states.size());
            auto moving_thres_per_one_frame = MovingThreshold * tick_s;
            auto planed_targets = joint_targets_->front();
            for (auto i = 0; i < motor_states.size(); ++i) {
                if (motor_states[i].control_mode == ControlModePosition) {
                    auto goal_angle = 0.0;
                    auto current_angle = motor_states[i].angle;
                    if (abs(planed_targets[i] - current_angle) < moving_thres_per_one_frame) {
                        goal_angle = current_angle;
                    } else if (planed_targets[i] > current_angle) {
                        goal_angle = current_angle + DecelerationDistance;
                    } else {
                        goal_angle = current_angle - DecelerationDistance;
                    }

                    for (auto j = 0; j < deceleration_times; ++j) {
                        auto target_angle = current_angle + (goal_angle - current_angle) * (static_cast<double>(j) / deceleration_times);
                        decelerated_results[i].emplace_back(target_angle);
                    }
                } else {
                    // TODO: 速度、トルク、電流制御の場合はいきなりゼロにしても問題がないはず
                    // 様子を観察して必要であれば、最終制御値から徐々にゼロに落とす形の実装を検討する
                    for (auto j = 0; j < deceleration_times; ++j) {
                        decelerated_results[i].emplace_back(0.0);
                    }
                }
            }
            SetJointArrayToTrajectory(decelerated_results, ref_array);
            // 所有権を移す
            joint_targets_ = move(ref_array);
            break;
    }
    lck.unlock();
}

void DDMControlService::dummyControlForUpdates(vector<double>& last_targets,
                                               const shared_ptr<vector<MotorState>>& mstate_vector) {
    // 状態更新
    servoControl_(last_targets, mstate_vector, true);

    // 初回のみ
    if (first_flag_) {
        // last_targetの更新
        for (auto i = 0; i < mstate_vector->size(); ++i) {
            switch (mstate_vector->at(i).control_mode) {
            case ControlModePosition:
                last_targets.at(i) = mstate_vector->at(i).angle;
                break;
            case ControlModeCurrent:
                last_targets.at(i) = mstate_vector->at(i).current;
                break;
            case ControlModeTorque:
                last_targets.at(i) = mstate_vector->at(i).torque;
                break;
            default:
                last_targets.at(i) = 0.0;
                break;
            }
        }
        first_flag_ = false;
    }
}

void DDMControlService::writeJointOffsetsToFile() {
    MotorParams mparams;
    YAML::Node offsets;
    for (auto i = 0; i < motor_controller_->getJointNum(); ++i) {
        motor_controller_->copyMotorParams(i, mparams);
        offsets[i] = mparams.joint_offset;
    }
    YAML::Emitter out;
    out << offsets;

    ofstream file(JOINT_CALIB_FILE);
    file << out.c_str();
    file.close();
}
