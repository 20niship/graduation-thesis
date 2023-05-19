# -*- coding: utf-8 -*-
import ctypes
import os
from ctypes import c_int, c_char_p, c_double, POINTER


hr4c_libdir = os.environ['HR4C_LIBDIR']
hr4c_comm_so = ctypes.cdll.LoadLibrary(hr4c_libdir + "/libhr4c_comm.so")
hr4c_comm_so.hr4capi_open.restype = c_int
hr4c_comm_so.hr4capi_open.argtypes = [c_char_p, c_int, c_int, c_char_p]
hr4c_comm_so.hr4capi_close.restype = c_int
hr4c_comm_so.hr4capi_close.argtypes = [c_int]
hr4c_comm_so.hr4capi_start.restype = c_int
hr4c_comm_so.hr4capi_start.argtypes = [c_int]
hr4c_comm_so.hr4capi_stop.restype = c_int
hr4c_comm_so.hr4capi_stop.argtypes = [c_int]
hr4c_comm_so.hr4capi_set_joint_reference.restype = None
hr4c_comm_so.hr4capi_set_joint_reference.argtypes = [c_int, POINTER(c_double), POINTER(c_int)]
hr4c_comm_so.hr4capi_set_joint_trajectory.restype = None
hr4c_comm_so.hr4capi_set_joint_trajectory.argtypes = [c_int, POINTER(c_double), c_double, c_int, POINTER(c_int)]
hr4c_comm_so.hr4capi_get_joint_angle.restype = None
hr4c_comm_so.hr4capi_get_joint_angle.argtypes = [c_int, POINTER(c_double)]
hr4c_comm_so.hr4capi_wait_interpolation.restype = None
hr4c_comm_so.hr4capi_wait_interpolation.argtypes = [c_int]
hr4c_comm_so.hr4capi_set_control_mode.restype = None
hr4c_comm_so.hr4capi_set_control_mode.argtypes = [c_int, POINTER(c_int)]
hr4c_comm_so.hr4capi_start_logging.restype = None
hr4c_comm_so.hr4capi_start_logging.argtypes = [c_int]
hr4c_comm_so.hr4capi_stop_logging.restype = None
hr4c_comm_so.hr4capi_stop_logging.argtypes = [c_int]
hr4c_comm_so.hr4capi_clear_logs.restype = None
hr4c_comm_so.hr4capi_clear_logs.argtypes = [c_int]
hr4c_comm_so.hr4capi_get_lognum.restype = c_int
hr4c_comm_so.hr4capi_get_lognum.argtypes = [c_int]
hr4c_comm_so.hr4capi_get_loglist.restype = None
hr4c_comm_so.hr4capi_get_loglist.argtypes = [c_int, c_char_p]
hr4c_comm_so.hr4capi_get_log.restype = c_int
hr4c_comm_so.hr4capi_get_log.argtypes = [c_int, c_char_p]
hr4c_comm_so.hr4capi_servo_on.restype = None
hr4c_comm_so.hr4capi_servo_on.argtypes = [c_int, POINTER(c_int), c_int]
hr4c_comm_so.hr4capi_servo_all_on.restype = None
hr4c_comm_so.hr4capi_servo_all_on.argtypes = [c_int]
hr4c_comm_so.hr4capi_servo_off.restype = None
hr4c_comm_so.hr4capi_servo_off.argtypes = [c_int, POINTER(c_int), c_int]
hr4c_comm_so.hr4capi_servo_all_off.restype = None
hr4c_comm_so.hr4capi_servo_all_off.argtypes = [c_int]
hr4c_comm_so.hr4capi_get_control_mode.restype = None
hr4c_comm_so.hr4capi_get_control_mode.argtypes = [c_int, POINTER(c_int)]
hr4c_comm_so.hr4capi_get_joint_current.restype = None
hr4c_comm_so.hr4capi_get_joint_current.argtypes = [c_int, POINTER(c_double)]
hr4c_comm_so.hr4capi_calibrate_joint.resttype = None
hr4c_comm_so.hr4capi_calibrate_joint.argtypes = [c_int, c_int, c_double]
hr4c_comm_so.hr4capi_calibrate_joint_from_memory.resttype = None
hr4c_comm_so.hr4capi_calibrate_joint_from_memory.argtypes = [c_int, c_int, c_double, c_double]
hr4c_comm_so.hr4capi_alarm_reset.restype = None
hr4c_comm_so.hr4capi_alarm_reset.argtypes = [c_int, POINTER(c_int), c_int]
hr4c_comm_so.hr4capi_get_motor_status.restype = None
hr4c_comm_so.hr4capi_get_motor_status.argtypes = [c_int, POINTER(c_int)]
hr4c_comm_so.hr4capi_force_stop.restype = None
hr4c_comm_so.hr4capi_force_stop.argtypes = [c_int, POINTER(c_int), c_int]
hr4c_comm_so.hr4capi_get_joint_speed.restype = None
hr4c_comm_so.hr4capi_get_joint_speed.argtypes = [c_int, POINTER(c_double)]
hr4c_comm_so.hr4capi_get_joint_torque.restype = None
hr4c_comm_so.hr4capi_get_joint_torque.argtypes = [c_int, POINTER(c_double)]
hr4c_comm_so.hr4capi_start_teaching.restype = c_int
hr4c_comm_so.hr4capi_start_teaching.argtypes = [c_int]
hr4c_comm_so.hr4capi_stop_teaching.restype = c_int
hr4c_comm_so.hr4capi_stop_teaching.argtypes = [c_int]
hr4c_comm_so.hr4capi_replay_motion.restype = c_int
hr4c_comm_so.hr4capi_replay_motion.argtypes = [c_int, c_int, POINTER(c_int)]
hr4c_comm_so.hr4capi_get_motion_list.restype = None
hr4c_comm_so.hr4capi_get_motion_list.argtypes = [c_int, c_char_p]
hr4c_comm_so.hr4capi_clear_motion.restype = c_int
hr4c_comm_so.hr4capi_clear_motion.argtypes = [c_int, c_int]
hr4c_comm_so.hr4capi_clear_all_motions.restype = None
hr4c_comm_so.hr4capi_clear_all_motions.argtypes = [c_int]
hr4c_comm_so.hr4capi_controller_shutdown.restype = None
hr4c_comm_so.hr4capi_controller_shutdown.argtypes = [c_int]
hr4c_comm_so.hr4capi_ping.restype = c_int
hr4c_comm_so.hr4capi_ping.argtypes = [c_int]
hr4c_comm_so.hr4capi_update_controller.restype = c_int
hr4c_comm_so.hr4capi_update_controller.argtypes = [c_int, c_char_p]
hr4c_comm_so.hr4capi_get_all_sensor_info.restype = None
hr4c_comm_so.hr4capi_get_all_sensor_info.argtypes = [c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double),
                                                     POINTER(c_double), POINTER(c_int), POINTER(c_int)]
hr4c_comm_so.hr4capi_enable_zerog_mode.restype = None
hr4c_comm_so.hr4capi_enable_zerog_mode.argtypes = [c_int, c_int]


