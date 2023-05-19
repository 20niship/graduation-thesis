#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <libhr4c_comm.h>

using namespace std;


bool prog_finish=false;

void sig_int(int signal){
  prog_finish=true;
}

class RobotHR4C
{
    static constexpr int INVALIDATE_INSTANCE_NO = -1;
    static constexpr int DOF = 6;
    int _instance_no;
    bool _is_connected;
public:
    RobotHR4C()
        :_instance_no(INVALIDATE_INSTANCE_NO)
        , _is_connected(false) {}

    RobotHR4C(const char* ip, int port, bool bStart=true)
        : _instance_no(INVALIDATE_INSTANCE_NO)
        , _is_connected(false) {
        open(ip, port);
        if (bStart) ready();
    }

    virtual ~RobotHR4C() {
        terminate();
    }

    bool validate() const {
        return (_instance_no >= 0);
    }

    bool isConnected() const {
        return _is_connected;
    }

    bool isReady() const {
        return (validate() && isConnected());
    }

    bool open(const char* ip, int port) {
        if (!validate())
            _instance_no = hr4capi_open(const_cast<char*>(ip), port, DOF, "green");
        return validate();
    }

    void close() {
        if (validate()) {
            hr4capi_close(_instance_no);
            _instance_no = INVALIDATE_INSTANCE_NO;
        }
    }

    bool ready() {
        if (validate() && !isConnected()) {
            _is_connected = (hr4capi_start(_instance_no) > 0);
            reset();
        }
        return isReady();
    }

    void terminate() {
        servoAllOff();
        if (isReady()) {
            hr4capi_stop(_instance_no);
            _is_connected = false;
        }
        close();
    }

    void reset() {
        if (!isReady()) return;

        int ctrl_mode[6] = {1,1,1,1,1,1};
        hr4capi_set_control_mode(_instance_no, ctrl_mode);

        int joint_nos[6] = {0,1,2,3,4,5};
        hr4capi_alarm_reset(_instance_no, joint_nos, 6);
    }

    void status() {
        int mt_status[6];
        hr4capi_get_motor_status(_instance_no, mt_status);
        std::cout<<"[motor]"<<std::endl;
        for(int i=0;i<6;i++){
            std::cout<<mt_status[i]<<" ";
        }
        std::cout<<std::endl;

        int ctl_mode[6];
        hr4capi_get_control_mode(_instance_no, ctl_mode);
        std::cout<<"[ctrl-mode]"<<std::endl;
        for(int i=0;i<6;i++){
            std::cout<<ctl_mode[i]<<" ";
        }
        std::cout<<std::endl;
    }

    void servoAllOn() {
        if (!isReady()) return;
        hr4capi_servo_all_on(_instance_no);
    }

    void servoAllOff() {
        if (!isReady()) return;
        hr4capi_servo_all_off(_instance_no);
    }

    void servoOn(const int* joint_nos, int size) {
        if (!isReady()) return;
        hr4capi_servo_on(_instance_no, const_cast<int*>(joint_nos), size);
    }

    void servoOff(const int* joint_nos, int size) {
        if (!isReady()) return;
        hr4capi_servo_off(_instance_no, const_cast<int*>(joint_nos), size);
    }

    void waitInterpolation() {
        if (!isReady()) return;
        hr4capi_wait_interpolation(_instance_no);
    }

    void setJointTrajectory(
        const double* goal_angles,
        double time,
        int method,
        const int* masks)
    {
        if (!isReady()) return;
        double target_angles[6];
        memcpy(target_angles, goal_angles, sizeof(double) * 6);
        double j2_angle = target_angles[1];
        double j3_angle = target_angles[2];
        target_angles[2] = j2_angle + j3_angle;

        hr4capi_set_joint_trajectory(_instance_no,
                                     target_angles,
                                     time,
                                     method,
                                     const_cast<int*>(masks));
    }

    void setJointReference(
        const double* joint_references,
        const int* masks)
    {
        if (!isReady()) return;
        double target_angles[6];
        memcpy(target_angles, joint_references, sizeof(double) * 6);
        double j2_angle = target_angles[1];
        double j3_angle = target_angles[2];
        target_angles[2] = j2_angle + j3_angle;

        hr4capi_set_joint_reference(_instance_no,
                                    target_angles,
                                    const_cast<int*>(masks));
    }

    void getJointAngle(double* joint_angles) const {
        if (!isReady()) return;
        hr4capi_get_joint_angle(_instance_no, joint_angles);
        double j2_angle = joint_angles[1];
        double j3_angle = joint_angles[2];
        joint_angles[2] = j3_angle - j2_angle;
    }

    void getAllSensorInfo(double* joint_angles, double* joint_current, double* joint_speed,
                          double* joint_torque, int* control_mode, int* motor_status) const {
        if (!isReady()) return;
        hr4capi_get_all_sensor_info(_instance_no,
                                    joint_angles, joint_current, joint_speed,
                                    joint_torque, control_mode, motor_status);
        double j2_angle = joint_angles[1];
        double j3_angle = joint_angles[2];
        joint_angles[2] = j3_angle - j2_angle;
    }
};


int main() {

    signal(SIGINT, sig_int);

    ofstream fout("ang.log");

    RobotHR4C robot1("172.16.1.24", 54321);
    if (!robot1.isReady()) {
        std::cout << "[error] failed robot1 initializing..." << std::endl;
        return -1;
    }

    RobotHR4C robot2("172.16.1.23", 54321);
    if (!robot2.isReady()) {
        std::cout << "[error] failed robot2 initializing..." << std::endl;
        return -1;
    }

    // servo on
    robot1.servoAllOn();
    robot2.servoAllOn();

    // move to start position
    int mask[6] = {0, 0, 0, 0, 0, 0};
    double ref0[6]={0, 1.57, 1.57, 0, 0, 0};
    robot1.setJointTrajectory(ref0, 3.0, 0, mask);
    robot2.setJointTrajectory(ref0, 3.0, 0, mask);
    robot1.waitInterpolation();
    robot2.waitInterpolation();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // wait for key input
    std::cout << "Press any key to start" << std::endl;
    getchar();
    robot1.servoAllOff();

    // base time
    struct timeval tv0;
    gettimeofday(&tv0, nullptr);

    // get timestamp function
    auto get_timestamp_ms = [&tv0]()->double {
        struct timeval tv1;
        gettimeofday(&tv1, nullptr);
        return (tv1.tv_sec-tv0.tv_sec)*1000 + (double)(tv1.tv_usec-tv0.tv_usec)/1000;
    };

    auto get_timestamp_us = [&tv0]()->double {
        struct timeval tv1;
        gettimeofday(&tv1, nullptr);
        return (tv1.tv_sec-tv0.tv_sec)*1000000 + (double)(tv1.tv_usec-tv0.tv_usec);
    };

    int ref_count=0;
    constexpr double EPSIRON = 0.0001;
    constexpr double LOOP_TIMELIMIT_MS = 5000;

    while(!prog_finish){
        double start_ms = get_timestamp_ms();

        // all sensor for robot1
        double joint_angles[6];
        double joint_current[6];
        double joint_speed[6];
        double joint_torque[6];
        int control_mode[6];
        int motor_status[6];
        robot1.getAllSensorInfo(joint_angles, joint_current, joint_speed,
                                joint_torque, control_mode, motor_status);

        // set joint angle to robot2
        robot2.setJointReference(joint_angles, mask);

        double end_ms = get_timestamp_ms();
        double elapsed = end_ms - start_ms;

        std::cout << elapsed  << " ms" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // wait for key input
    std::cout << "Press any key to finish" << std::endl;
    getchar();
    robot2.servoAllOff();

    return 0;
}
