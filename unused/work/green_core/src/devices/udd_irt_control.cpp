#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <utility>
#include <thread>
#include <spdlog/spdlog.h>
#include "devices/udd_irt_control.h"

using namespace hr4c::devices;
using namespace std;


template<typename T>
void makeTargetMessage(T target, vector<unsigned char>& msg) {
    // 元の型、それぞれ特殊化させて対応
}

template<>
void makeTargetMessage(int target, vector<unsigned char>& msg) {
    // pos, int32
    // 位置指令値 little endien
    union {
        int i32;
        unsigned char uc[sizeof(float)];
    } uval{};

    uval.i32 = target;

    for (unsigned char& uc : uval.uc) {
        msg.push_back(uc);
    }
}

template<>
void makeTargetMessage(float target, vector<unsigned char>& msg) {
    // float IEEE754 32bit
    // 指令値 little endien
    union {
        float f;
        unsigned char uc[sizeof(float)];
    } uval{};

    uval.f = target;

    for (unsigned char& uc : uval.uc) {
        msg.push_back(uc);
    }
}

template<typename T>
void makeMessage(int bid, int servocmd, T target, int control_mode, int spi_error_reset, int alarm_reset,
                 vector<unsigned char>& msg) {
    // +0 b7-4  ドライバーID
    // b3    サーボ指令
    // b2-0  制御モード切替指令
    // +1 b7-0  指令速度 IEEE754 単精度              0-7bit
    //                          +2 b7-0  指令速度   8-15bit
    // +3 b7-0  指令速度                           16-23bit
    // +4 b7-0  指令速度                           24-31bit
    // +5 b7-5 予約 (0固定)
    // b4 SPI通信エラーリセット
    // b3 DIN3
    // b2 DIN2
    // b1 DIN1
    // b0 DIN0
    msg.clear();
    unsigned char val = (unsigned char) bid << 4 | (unsigned char) servocmd << 3 | (unsigned char) control_mode;
    msg.push_back(val); // id, servocmd, 000
    makeTargetMessage<T>(target, msg);
    unsigned char err_rest = ((unsigned char)spi_error_reset & 0x01) << 4 | (unsigned char) alarm_reset;
    msg.push_back(err_rest);
}

void parseMessage(vector<unsigned char>& rxvec, unsigned long pulse_per_round, double gear_ratio, double gear_efficiency,
                  double torque_constant, int offset, int motor_dir, MotorState& mstate, double tick_s, int seq_no)
{
    union {
        float f;
        unsigned char uc[sizeof(float)];
    } uval{};

    if (!rxvec.empty()) {
        if (rxvec[0] == 0xff) {
            bool connection_error_flag = true;
            for (unsigned char& rx : rxvec){
                if (rx != 0xff) {
                    connection_error_flag = false;
                    break;
                }
            }
            if (connection_error_flag) {
                spdlog::error("connection error @" + to_string(mstate.did) + "!");
                mstate.connection_error = 1;
            } else {
                spdlog::error("checksum error@" + to_string(mstate.did) + "!");
            }
            return;
        }

        mstate.did = (int)((rxvec[0] & 0xf0) >> 4);
        mstate.servo_state = (int)((rxvec[0] & 0x08) >> 3);
        mstate.alarm = (int)((rxvec[0] & 0x04) >> 2);
        mstate.limit_alarm = (int)((rxvec[0] & 0x02) >> 1);
        mstate.device_error = (int)(rxvec[0] & 0x01);
        if (mstate.device_error) {
            spdlog::error("spi error@" + to_string(mstate.did));
        }

        mstate.encoder = (int32_t)rxvec[1] | (int32_t)rxvec[2] << 8 | (int32_t)rxvec[3] << 16 | (int32_t)rxvec[4] << 24;
        uval.uc[0] = rxvec[5];
        uval.uc[1] = rxvec[6];
        uval.uc[2] = rxvec[7];
        uval.uc[3] = rxvec[8];
        mstate.current = static_cast<float>(convertCurrentDirection(uval.f, motor_dir));
        mstate.finished = (rxvec[9] & 0x80) >> 7;

        auto cmode = (rxvec[9] & 0x0f);
        if(mstate.control_mode == ControlModeTorque && cmode == ControlModeCurrent) {
            // do nothing
        } else {
            mstate.control_mode = cmode;
        }

        auto prev_angle = mstate.angle;
        mstate.angle = convertPulse2Angle(pulse_per_round, gear_ratio, mstate.encoder, offset, motor_dir);
        if (tick_s > 0) {
            // 通常周期での更新の場合のみ速度は更新する
            mstate.speed = (mstate.angle - prev_angle) / tick_s;
        }
        mstate.torque = mstate.current * gear_ratio * gear_efficiency * torque_constant;
        mstate.connection_error = 0;
        mstate.seq_no = seq_no;
    } else {
        spdlog::error("Error - rxvec is empty");
        mstate.connection_error = 1;
    }
}

UDDIRTControl::UDDIRTControl(const YAML::Node& joint_config) {
    // open UDP socket
    auto controller_ip = joint_config["controller_ip"].as<string>();
    auto controller_send_port = joint_config["controller_send_port"].as<int>();
    auto controller_recv_port = joint_config["controller_recv_port"].as<int>();
    udp_client_ = make_shared<UdpClient>(controller_ip, controller_send_port, controller_recv_port);
    auto joint_names = joint_config["joint_names"].as<vector<string>>();
    dof_= joint_names.size();
    last_position_targets_.resize(dof_, -1);
}

int UDDIRTControl::openDevice()
{
    udp_client_->open();

    return 0;
}

int UDDIRTControl::closeDevice() {
    // do nothing
    return 0;
}

void UDDIRTControl::makeSendTelegram_(vector<unsigned char>& sendbuf) {
    auto cnt = 0;
    auto send_datalen = 2 + 6 * SEND_BYTES;
    auto sendlen = send_datalen + 6;
    auto checksum = 0;
    static int seq_no = 0;

    sendbuf.resize(sendlen);
    sendbuf[0] = 0x02;
    sendbuf[1] = 0xff;
    sendbuf[2] = 0xff;
    seq_no += 1;
    if (seq_no > 255) {
        seq_no = 0;
    }
    sendbuf[3] = seq_no;
    sendbuf[4] = (send_datalen & 0xff00) >> 8;
    sendbuf[5] = (send_datalen & 0x00ff);
    sendbuf[6] = 0x02;
    unsigned char motor_enable = 0x00;
    for (auto jn = 0; jn < getJointNum(); ++jn) {
        motor_enable |= 0x01 << jn;
    }
    sendbuf[7] = motor_enable;
    for (auto& tx : tx_data_) {
        copy_n(tx.begin(), SEND_BYTES, sendbuf.begin() + 8 + cnt * SEND_BYTES);
        cnt += 1;
    }
}

void UDDIRTControl::setMessage_(int joint_no, int bid, int servocmd,
                                int target_int, float target_float, int control_mode,
                                int spi_error_reset, int alarm_reset,
                                bool set_control_mode, bool control_int_mode) {
    vector<unsigned char> msg;

    if (set_control_mode) {
        if (control_int_mode) {
            makeMessage<int>(bid, servocmd, target_int, control_mode, spi_error_reset, alarm_reset, msg);
        } else {
            makeMessage<float>(bid, servocmd, target_float, control_mode, spi_error_reset, alarm_reset, msg);
        }
    } else {
        switch (control_mode) {
            case ControlModePosition:
                makeMessage<int>(bid, servocmd, target_int, ControlModeNochange, spi_error_reset, alarm_reset, msg);
                break;
            case ControlModeCurrent:
            case ControlModeTorque:
            case ControlModeSpeed:
                makeMessage<float>(bid, servocmd, target_float, ControlModeNochange, spi_error_reset, alarm_reset, msg);
                break;
            default:
                makeMessage<int>(bid, servocmd, target_int, ControlModeNochange, spi_error_reset, alarm_reset, msg);
                break;
        }
    }
    copy_n(msg.begin(), SEND_BYTES, tx_data_[joint_no].begin());
}

void UDDIRTControl::writeMessage_() {
    vector<unsigned char> sendbuf, recvbuf;

    // write data
    makeSendTelegram_(sendbuf);
    auto sendlen = udp_client_->sendAllData((const unsigned char*)&sendbuf[0], sendbuf.size());
    if (sendlen <= 0) {
        spdlog::error("Failed to write");
        return;
    }

    // read data
    auto recv_datalen = 2 + 6 * RECV_BYTES;
    auto recvlen = recv_datalen + 6;
    recvbuf.resize(recvlen);
    auto actual_len = udp_client_->receiveAllData((unsigned char *)&recvbuf[0], recvlen);

    if (actual_len <= 0) {
        spdlog::error("Failed to read");
        return;
    } else if (recvbuf[0] != 0x02) {
        spdlog::error("Invalid header in received data");
        return;
    } else if (recvbuf[1] != 0xff) {
        spdlog::error("Invalid code in did");
        return;
    } else if (recvbuf[2] != 0xff) {
        spdlog::error("Invalid code in sid");
        return;
    } else if (recvbuf[6] != 0x02) {
        spdlog::error("Invalid response in ANS_STT");
        return;
    } else if (recvbuf[7] != 0x06) {
        spdlog::error("Invalid response in received data");
        return;
    }

    int calc_datlen = (static_cast<int>(recvbuf[4]) << 8) + static_cast<int>(recvbuf[5]);
    if (calc_datlen != recv_datalen) {
        spdlog::error("Invalid length in received data");
        return;
    }
    seq_no_ = recvbuf[3];
    auto cnt = 0;
    for (auto i = 0; i < getJointNum(); ++i) {
        rx_data_[i].fill(0);
        copy_n(recvbuf.begin() + 6 + 2 + cnt * RECV_BYTES, RECV_BYTES, rx_data_[i].begin());
        cnt += 1;
    }
}

void UDDIRTControl::updateMotorState_(int joint_no, const MotorParams& mparams, MotorState& mstate) {
    vector<unsigned char> resp;
    auto seq_no = readMessage_(joint_no, resp);
    chrono::system_clock::time_point now = chrono::system_clock::now();
    auto elapsed_ns = chrono::duration_cast<std::chrono::nanoseconds>(now - last_updated_).count();
    parseMessage(resp,
                 mparams.pulse_per_round,
                 mparams.gear_ratio,
                 mparams.gear_efficiency,
                 mparams.torque_constant,
                 mparams.joint_offset,
                 mparams.motor_direction,
                 mstate, elapsed_ns / 1000000000.0, seq_no);
    last_updated_ = chrono::system_clock::now();
}

bool UDDIRTControl::isOpen() {
    return udp_client_->isOpen();
}

int UDDIRTControl::readMessage_(int joint_no, vector<unsigned char>& resp) {
    resp.resize(RECV_BYTES);
    copy_n(rx_data_[joint_no].begin() , RECV_BYTES, resp.begin());
    return seq_no_;
}

void UDDIRTControl::servoOn(int joint_no, const MotorParams& mparams, MotorState& mstate) {
    auto bid = mparams.b_id;
    int target_pos = mstate.encoder;
    float target_ref;
    if (mstate.control_mode == ControlModePosition) {
        last_position_targets_.at(joint_no) = mstate.encoder;
    } else if (mstate.control_mode == ControlModeCurrent || mstate.control_mode == ControlModeTorque) {
        target_ref = mstate.current;
    } else {
        // 速度制御の場合は0.0が目標値となる
        target_ref = 0.0;
    }

    for (int i=0; i < 4; i++) {
        setMessage_(joint_no, bid, 1, target_pos, target_ref, mstate.control_mode, 0, 0);
        writeMessage_();
        updateMotorState_(joint_no, mparams, mstate);
        this_thread::sleep_for(chrono::nanoseconds(10 * 1000 * 1000));
    }
}

void UDDIRTControl::servoOnAll(const vector<MotorParams>& mparams_v,
                               vector<MotorState>& mstate_array) {
    for (int i=0; i < 4; i++) {
        for(int jn=0; jn < getJointNum(); ++jn) {
            int target_pos = mstate_array[jn].encoder;
            float target_ref;
            if (mstate_array[jn].control_mode == ControlModePosition) {
                last_position_targets_.at(jn) = mstate_array[jn].encoder;
                target_ref = 0.0;
            } else if (mstate_array[jn].control_mode == ControlModeCurrent || mstate_array[jn].control_mode == ControlModeTorque) {
                target_ref = mstate_array[jn].current;
            } else {
                // 速度制御の場合は0.0が目標値となる
                target_ref = 0.0;
            }
            auto bid = mparams_v.at(jn).b_id;
            setMessage_(jn, bid, 1, target_pos, target_ref, mstate_array[jn].control_mode, 0, 0);
        }
        writeMessage_();

        for(int jn=0; jn < getJointNum(); ++jn) {
            updateMotorState_(jn, mparams_v.at(jn), mstate_array.at(jn));
        }
        this_thread::sleep_for(chrono::nanoseconds(10 * 1000 * 1000));
    }
}

void UDDIRTControl::servoOff(int joint_no, const MotorParams& mparams, MotorState& mstate) {
    auto bid = mparams.b_id;
    int target_pos = mstate.encoder;
    // 電流制御および速度制御の場合は0.0が目標値となる
    float target_ref = 0.0;

    for (int i=0; i < 6; i++) {
        setMessage_(joint_no, bid, 0, target_pos, target_ref, mstate.control_mode, 0, 0);
        writeMessage_();
        updateMotorState_(joint_no, mparams, mstate);
        this_thread::sleep_for(chrono::nanoseconds(10 * 1000 * 1000));
    }
}

void UDDIRTControl::servoOffAll(const vector<MotorParams>& mparams_v, vector<MotorState>& mstate_array) {
    for (int i=0; i < 6; i++) {
        for(int jn=0; jn < getJointNum(); ++jn) {
            int target_pos = mstate_array[jn].encoder;
            // 電流制御および速度制御の場合は0.0が目標値となる
            float target_ref = 0.0;
            auto bid = mparams_v.at(jn).b_id;
            setMessage_(jn, bid, 0, target_pos, target_ref, mstate_array[jn].control_mode, 0, 0);
        }
        writeMessage_();

        for(int jn=0; jn < getJointNum(); ++jn) {
            updateMotorState_(jn, mparams_v.at(jn), mstate_array.at(jn));
        }
        this_thread::sleep_for(chrono::nanoseconds(10 * 1000 * 1000));
    }
}

void UDDIRTControl::setControlMode(vector<int>& c_modes,
                                   const vector<MotorParams>& mparams_v,
                                   vector<MotorState>& mstate_array) {
    auto joint_num = getJointNum();
    int32_t command;
    float command_ref = 0.0;

    for (int i = 0; i < joint_num; ++i) {
        if (mstate_array[i].control_mode != c_modes[i]) {
            if (mstate_array[i].control_mode == ControlModeCurrent && c_modes[i] == ControlModeTorque) {
                // 現在の値が電流制御かつ変更する制御モードがトルク制御の場合はアンプ側は変わらないためコントローラには何も送らないが、
                // モータ制御モードのみ変更する
                mstate_array[i].control_mode = ControlModeTorque;
            } else {
                // モード変更のときはコマンドの扱いが特殊になるので、注意
                auto mparams = mparams_v.at(i);
                auto control_mode = c_modes[i];
                mstate_array[i].control_mode = control_mode;
                switch (control_mode) {
                    case ControlModePosition:
                        command = mstate_array[i].encoder;
                        last_position_targets_.at(i) = mstate_array[i].encoder;
                        setMessage_(i, mparams.b_id, mstate_array[i].servo_state, command, command_ref,
                                    control_mode, 0, 0, true, true);
                        break;
                    case ControlModeCurrent:
                        command_ref = mstate_array[i].current;
                        setMessage_(i, mparams.b_id, mstate_array[i].servo_state, command, command_ref,
                                    control_mode, 0, 0, true, false);
                        break;
                    case ControlModeTorque:
                        command_ref = mstate_array[i].current;
                        control_mode = ControlModeCurrent;
                        mstate_array[i].control_mode = ControlModeTorque;
                        setMessage_(i, mparams.b_id, mstate_array[i].servo_state, command, command_ref,
                                    control_mode, 0, 0, true, false);
                        break;
                    case ControlModeSpeed:
                        command_ref = 0.0;
                        setMessage_(i, mparams.b_id, mstate_array[i].servo_state, command, command_ref,
                                    control_mode, 0, 0, true, false);
                        break;
                    default:
                        spdlog::error("Error: invalid control mode");
                        break;
                }
            }
        }
    }
    writeMessage_();
}

void UDDIRTControl::servoControlSetMessage_(int joint_no, vector<double>& targets, const MotorParams& mparams,
                                            vector<MotorState>& mstate_array, bool dummy_flag) {
    vector<unsigned char> tgt;
    vector<unsigned char> resp;
    int target_pos;
    float target_ref;
    double target_angle, max_angle, min_angle;
    auto last_target = last_position_targets_.at(joint_no);

    switch (mstate_array[joint_no].control_mode) {
        case ControlModePosition:
            if (!dummy_flag || last_target == -1) {
                // 角度の最大最小値チェック
                max_angle = mparams.max_angle;
                min_angle = mparams.min_angle;
                target_angle = minMaxCheck(targets[joint_no], mstate_array[joint_no].angle,
                                           min_angle, max_angle, mstate_array[joint_no].control_mode,
                                           mparams.b_id);
                // パルスへ変換
                target_pos = convertAngle2Pulse(mparams.pulse_per_round,
                                                mparams.gear_ratio,
                                                target_angle,
                                                mparams.joint_offset,
                                                mparams.motor_direction);
            } else {
                target_pos = last_target;
            }
            break;
        case ControlModeCurrent:
            target_ref = static_cast<float>(convertCurrentDirection(targets[joint_no],
                                                                    mparams.motor_direction));
            break;
        case ControlModeTorque:
            target_ref = static_cast<float>(convertTorque2Current(mparams.torque_constant,
                                                                  mparams.gear_efficiency,
                                                                  mparams.gear_ratio,
                                                                  targets[joint_no],
                                                                  mparams.motor_direction));
            break;
        case ControlModeSpeed:
            target_ref = static_cast<float>(convertRadPerSec2RPM(mparams.gear_ratio,
                                                                 targets[joint_no],
                                                                 mparams.motor_direction));
            break;
        default:
            if (!dummy_flag) {
                target_pos = convertAngle2Pulse(mparams.pulse_per_round,
                                                mparams.gear_ratio,
                                                targets[joint_no],
                                                mparams.joint_offset,
                                                mparams.motor_direction);
            } else {
                // 状態取得の場合は現在のエンコーダ値を制御目標とする
                target_pos = mstate_array[joint_no].encoder;
            }
            break;
    }

    if(isOpen()) {
        auto bid = mparams.b_id;
        setMessage_(joint_no, bid, mstate_array[joint_no].servo_state, target_pos, target_ref,
                    mstate_array[joint_no].control_mode, 0, 0);
    }
}

void UDDIRTControl::sendTargetsAndUpdateStates(vector<double>& targets,
                                               const vector<MotorParams>& mparams_v,
                                               vector<MotorState>& mstate_array,
                                               double tick_s,
                                               bool dummy_flag) {
    // 使用する軸の分を一気にデータセット
    for( int i = 0; i < getJointNum(); i++) {
        servoControlSetMessage_(i, targets, mparams_v.at(i), mstate_array, dummy_flag);
        if (!dummy_flag) {
            switch (mstate_array.at(i).control_mode) {
                case ControlModePosition:
                    last_position_targets_.at(i) = convertAngle2Pulse(mparams_v.at(i).pulse_per_round,
                                                                      mparams_v.at(i).gear_ratio,
                                                                      targets.at(i),
                                                                      mparams_v.at(i).joint_offset,
                                                                      mparams_v.at(i).motor_direction);
                    break;
                default:
                    // 他のモードでは使用しないため-1をセットする
                    last_position_targets_.at(i) = -1;
                    break;
            }
        }
    }

    // 書き込み
    writeMessage_();

    // データ更新
    chrono::system_clock::time_point now = chrono::system_clock::now();
    for( int i = 0; i < getJointNum(); i++) {
        updateMotorState_(i, mparams_v.at(i), mstate_array.at(i));
    }
    last_updated_ = chrono::system_clock::now();
}

void UDDIRTControl::alarmReset(int joint_no, const MotorParams& mparams, MotorState& mstate) {
    int target_pos = mstate.encoder;
    // 電流制御および速度制御の場合は0.0が目標値となる
    float target_ref = 0.0;

    for (int i = 0; i < 3; i++) {
        auto bid = mparams.b_id;
        setMessage_(joint_no, bid, 0, target_pos, target_ref, mstate.control_mode,0, 1);
        writeMessage_();
        updateMotorState_(joint_no, mparams, mstate);
        this_thread::sleep_for(chrono::nanoseconds(10 * 1000 * 1000));
    }
}

double UDDIRTControl::calibrateJoint(double calibrate_angle,
                                     const MotorParams& mparams,
                                     const MotorState& mstate) {
    auto calibrate_pulse = convertAngle2Pulse(mparams.pulse_per_round,
                                              mparams.gear_ratio,
                                              calibrate_angle,
                                              0.0,
                                              mparams.motor_direction);

    return (mstate.encoder - calibrate_pulse);
}

double UDDIRTControl::calibrateJointFromMemory(double calibrate_angle,
                                               double memory_angle,
                                               const MotorParams &mparams,
                                               const MotorState &mstate) {
    auto memory_pulse = convertAngle2Pulse(mparams.pulse_per_round,
                                           mparams.gear_ratio,
                                           memory_angle,
                                           mparams.joint_offset,
                                           mparams.motor_direction);

    auto calibrate_pulse = convertAngle2Pulse(mparams.pulse_per_round,
                                              mparams.gear_ratio,
                                              calibrate_angle,
                                              0.0,
                                              mparams.motor_direction);

    return (memory_pulse - calibrate_pulse);
}
