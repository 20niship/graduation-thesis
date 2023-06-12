#include <libhr4c_comm.h>
#include <iostream>
#include <signal.h>
#include <fstream>
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std;


bool prog_finish=false;
void sig_int(int signal){
  prog_finish=true;
}

class RobotHR4C
{
    static constexpr int OPEN_TIMEOUT_S = 1;
    static constexpr int INVALIDATE_INSTANCE_NO = -1;
    static constexpr int DOF = 6;
    int _instance_no;
    bool _is_connected;
public:
    RobotHR4C()
        :_instance_no(INVALIDATE_INSTANCE_NO)
        ,_is_connected(false)
    {}
    RobotHR4C(const char* ip, int port, bool bStart=true)
        : _instance_no(INVALIDATE_INSTANCE_NO)
        , _is_connected(false)
    {
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
            _instance_no = hr4capi_open(const_cast<char*>(ip), port, OPEN_TIMEOUT_S, DOF);
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

    void setJointTrajectory(
        const double* goal_angles,
        double time,
        int method,
        const int* masks)
    {
        if (!isReady()) return;
        hr4capi_set_joint_trajectory(_instance_no,
            const_cast<double*>(goal_angles),
            time,
            method,
            const_cast<int*>(masks));
    }

    void setJointReference(
        const double* joint_references,
        const int* masks)
    {
        if (!isReady()) return;
        hr4capi_set_joint_reference(_instance_no,
            const_cast<double*>(joint_references),
            const_cast<int*>(masks));
    }

    void getJointAngle(double* joint_angles) const {
        if (!isReady()) return;
        hr4capi_get_joint_angle(_instance_no, joint_angles);
    }

    bool interpolationing() const {
        if (!isReady()) return false;
        return (hr4capi_check_interpolation(_instance_no) == 1);
    }
};


int main() {

    signal(SIGINT,sig_int);

    ofstream fout("ang.log");

    // ready robot
    auto ready = [](RobotHR4C& robot, bool b_servo=true, bool b_pose=true)->bool {
        if (!robot.isReady())
            return false;
        robot.status();

        int mask_init[6]={0,0,0,0,0,0};
        double joint_init[6];
        robot.getJointAngle(joint_init);
        robot.setJointReference(joint_init, mask_init);

        if (b_servo) {
            robot.servoAllOn();
            if (b_pose) {
                // move to start position
                int mask[6]={0,0,0,0,0,0};
                double ref[6]={1.54, 1.507, 3.046, 0, 0, 0};
                robot.setJointTrajectory(ref, 3.0, 0, mask);
            }
        }
        return true;
    };
    RobotHR4C robot_master("172.16.1.24", 54321);
    if (!ready(robot_master, false)) {
        std::cout << "[error] failed robot(master) initializing..." << std::endl;
        return -1;
    }
    RobotHR4C robot_slave("172.16.1.23", 54321);
    if (!ready(robot_slave)) {
        std::cout << "[error] failed robot(slave) initializing..." << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "start" << std::endl;

    // base time
    struct timeval tv0;
    gettimeofday(&tv0, nullptr);
    // get timeatamp function
    auto get_timestamp_ms = [&tv0]() {
        struct timeval tv1;
        gettimeofday(&tv1, nullptr);
        return (tv1.tv_sec-tv0.tv_sec)*1000 + (double)(tv1.tv_usec-tv0.tv_usec)/1000;
    };

    // control values
    int mask[6] = {0,0,0,0,0,0};
    constexpr double MOVE_LIMIT_JOINT0 = 0.1;

    // save slave joint
    double joint_save[6];
    robot_slave.getJointAngle(joint_save);
    // limit joint function
    auto limit_joint = [](double* next, double* cur) {
        // joint0 only
        double d = next[0] - cur[0];
        if (std::abs(d) >= MOVE_LIMIT_JOINT0)
            next[0] = (d < 0)
                ? cur[0] - MOVE_LIMIT_JOINT0
                : cur[0] + MOVE_LIMIT_JOINT0;
    };

    while(!prog_finish){

        double start_ms = get_timestamp_ms();

        // slave proc
        double joint_new[6];
        robot_master.getJointAngle(joint_new);
        limit_joint(joint_new, joint_save);
        robot_slave.setJointReference(joint_new, mask);
        for (auto i=0; i<6; ++i)
            joint_save[i] = joint_new[i];

        // interval
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // move to safety position
    double joint_safety_angles[6] = {1.54, 1.507, 3.046, 0, 0, 0};
    robot_slave.setJointTrajectory(joint_safety_angles, 4.0, 0, mask);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;
}
