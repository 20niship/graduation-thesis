#ifndef HR4C_CORE_DDM_CONTROL_SERVICE_H
#define HR4C_CORE_DDM_CONTROL_SERVICE_H
#include <condition_variable>
#include <deque>
#include <mutex>
#include "motor_controller/motor_state.h"
#include "motor_controller/motor_control.h"


namespace hr4c::services {
    using namespace devices;
    constexpr auto MethodSuddenStop = 0;
    constexpr auto MethodDeceleration = 1;

    // 減速停止時間
    constexpr auto DecelerationSec = 2.0;
    // 減速停止距離（radian)
    constexpr auto DecelerationDistance = 0.2;
    // 減速停止移動判別閾値（radian/s)
    constexpr auto MovingThreshold = 0.0872;

    // モータ同期制御クラス
    class DDMControlService {
    public:
        explicit DDMControlService(const shared_ptr<MotorControl>& motor_controller);
        ~DDMControlService() = default;
        void shutdownDDMControllers();
        void controlDDM(vector<double>& last_targets, shared_ptr<vector<MotorState>>& mstate_vector);
        void servoOn(int joint_no, vector<double>& last_targets, MotorState& mstate);
        void servoAllOn(vector<double>& last_targets, vector<MotorState>& mstate_vector);
        void servoOff(int joint_no, MotorState& mstate);
        void servoAllOff(vector<MotorState>& mstate_vector);
        void setControlModeWhenReady(vector<int>& c_modes, vector<MotorState>& mstate_vector, vector<double>& last_targets);
        void setJointTargetsWhenReady(const shared_ptr<deque<vector<double> > >& targets,
                                      const shared_ptr<vector<int> >& masks);
        void intervalSleep(long tick_ns, chrono::system_clock::time_point start);
        void waitInterpolation();
        void calibrateJointWhenReady(int joint_no, double calibrate_angle, MotorState& mstate);
        void calibrateJointFromMemoryWhenReady(int joint_no, double calibrate_angle, double memory_angle, MotorState& mstate);
        void alarmReset(int joint_no, double current_reference, MotorState& mstate);
        void forceStop(int method, vector<MotorState>& motor_states, double tick_s);
        void dummyControlForUpdates(vector<double>& last_targets, const shared_ptr<vector<MotorState>>& mstate_vector);
    private:
        mutex mtx_;
        bool first_flag_;
        bool wait_interpolation_flag_;
        shared_ptr<MotorControl> motor_controller_;
        shared_ptr<deque<vector<double> > > joint_targets_;
        shared_ptr<vector<int> > masks_;
        chrono::system_clock::time_point last_updated_;
        void servoControl_(vector<double>& targets, const shared_ptr<vector<MotorState>>& mstate_vector, bool dummy_flag);
        void writeJointOffsetsToFile();
    };
} // namespace hr4c::services
#endif //HR4C_CORE_DDM_CONTROL_SERVICE_H
