#pragma once
#include <iostream>

#include <hr4c/core/AxisInterface.hpp>
#include <hr4c/core/robotbase.hpp>
#include "../common/communication_const.hpp"

namespace hr4c {

bool start(const std::string& ip, int port);
bool terminate();
bool ping();

void hr4capi_set_joint_reference(int, double*, int*);
void hr4capi_set_joint_trajectory(int, double*, double, int, int*);
void hr4capi_wait_interpolation(int);
void hr4capi_set_control_mode(int, int*);
void hr4capi_start_logging(int);
void hr4capi_stop_logging(int);
void hr4capi_clear_logs(int);
int hr4capi_get_lognum(int);
void hr4capi_get_loglist(int, char*);
int hr4capi_get_log(int, const char*);
void hr4capi_servo_on(int, int*, int);
void hr4capi_servo_all_on(int);
void hr4capi_servo_off(int, int*, int);
void hr4capi_servo_all_off(int);
void hr4capi_get_control_mode(int, int*);
void hr4capi_get_joint_current(int, double*);
void hr4capi_calibrate_joint(int, int, double);
void hr4capi_calibrate_joint_from_memory(int, int, double, double);
void hr4capi_alarm_reset(int, int*, int);
void hr4capi_get_motor_status(int, int*);
void hr4capi_force_stop(int, int*, int);
void hr4capi_get_joint_speed(int, double*);
void hr4capi_get_joint_torque(int, double*);
int hr4capi_start_teaching(int);
int hr4capi_stop_teaching(int);
int hr4capi_replay_motion(int, int, int*);
void hr4capi_get_motion_list(int, char*);
int hr4capi_clear_motion(int, int);
void hr4capi_clear_all_motions(int);
void hr4capi_controller_shutdown(int);
int hr4capi_update_controller(int, const char*);
void hr4capi_get_all_sensor_info(int, double*, double*, double*, double*, int*, int*);
void hr4capi_enable_zerog_mode(int, int);

} // namespace hr4c
