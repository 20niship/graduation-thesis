#ifndef GREEN_CORE_UDD_IRT_CONTROL_H
#define GREEN_CORE_UDD_IRT_CONTROL_H
#include <termios.h>
#include <string>
#include <map>
#include <vector>
#include <unordered_map>
#include <motor_controller/motor_control.h>
#include <motor_controller/motor_state.h>
#include <net/udp_client.h>


namespace hr4c::devices {
    using namespace std;
    using namespace hr4c::udp_client;
    constexpr auto SEND_BYTES = 6;
    constexpr auto RECV_BYTES = 10;

    class UDDIRTControl {
     public:
        explicit UDDIRTControl(const YAML::Node& joint_config);
        UDDIRTControl() = delete;
        int openDevice ();
        int closeDevice ();
        [[nodiscard]] bool isOpen();
        void setControlMode(vector<int>& c_modes,
                            const vector<MotorParams>& mparams_v,
                            vector<MotorState>& mstate_array);
        void servoOn(int joint_no, const MotorParams& mparams, MotorState& mstate);
        void servoOnAll( const vector<MotorParams>& mparams_v, vector<MotorState>& mstate_array);
        void servoOff(int joint_no, const MotorParams& mparams, MotorState& mstate);
        void servoOffAll(const vector<MotorParams>& mparams_v, vector<MotorState>& mstate_array);
        void sendTargetsAndUpdateStates(vector<double>& targets,
                                        const vector<MotorParams>& mparams_v,
                                        vector<MotorState>& mstate_array,
                                        double tick_s,
                                        bool dummy_flag);
        void alarmReset(int joint_no, const MotorParams& mparams, MotorState& mstate);
        double calibrateJoint(double calibrate_angle, const MotorParams& mparams, const MotorState& mstate);
        double calibrateJointFromMemory(double calibrate_angle, double memory_angle,
                                        const MotorParams& mparams, const MotorState& mstate);
        int getJointNum() const { return dof_; }
     private:
        void makeSendTelegram_(vector<unsigned char>& sendbuf);
        void setMessage_(int joint_no, int bid, int servocmd,
                         int target_int, float target_float, int control_mode,
                         int spi_error_reset, int alarm_reset,
                         bool set_control_mode = false, bool control_int_mode = true);
        void writeMessage_();
        int readMessage_(int joint_no, vector<unsigned char>& resp);
        void updateMotorState_(int joint_no, const MotorParams& mparams, MotorState& mstate);
        void servoControlSetMessage_(int joint_no, vector<double>& targets, const MotorParams& mparams,
                                     vector<MotorState>& mstate_array, bool dummy_flag);
        shared_ptr<UdpClient> udp_client_;
        array<array<unsigned char, 6>, 6> tx_data_{};
        array<array<unsigned char, 10>, 6> rx_data_{};
        vector<int> last_position_targets_;
        int seq_no_{};
        int dof_{};
        chrono::system_clock::time_point last_updated_;
    };
} // namespace hr4c::devices
#endif //GREEN_CORE_UDD_IRT_CONTROL_H
