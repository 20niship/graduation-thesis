#ifndef HR4C_CORE_MOTOR_STATE_H
#define HR4C_CORE_MOTOR_STATE_H

namespace hr4c::devices {
    constexpr auto ControlModeNochange = 0;
    constexpr auto ControlModePosition = 1;
    constexpr auto ControlModeSpeed = 2;
    constexpr auto ControlModeCurrent = 3;
    constexpr auto ControlModeTorque = 4;

    class MotorState {
    public:
        MotorState() = default;
        ~MotorState() = default;

        int did;
        int servo_state;
        int alarm;
        int limit_alarm;
        int encoder;
        double angle;
        double raw_angle;
        float current;
        int finished;
        int device_error;
        int connection_error;
        int control_mode;
        double speed;
        double torque;
        int seq_no;
    };
} // namespace hr4c::devices

#endif //HR4C_CORE_MOTOR_STATE_H
