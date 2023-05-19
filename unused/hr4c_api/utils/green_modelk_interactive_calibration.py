#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import GreenModelK


class AutoCalibration(object):
    def __init__(self, ipaddr):
        self.robot = GreenModelK(ipaddr)
        # キャリブレーションオフセット
        self.calib_list = [2.26893, -1.0472, -0.6552, -2.0944, -2.1816, 0]
        if ipaddr == '172.16.1.21':
            self.calib_list[4] = -2.2283

    def _make_red_text(self, txt):
        return '\033[31m' + txt + '\033[0m'

    def run(self):
        print("start interactive calibration")

        # 初期姿勢移行時の上下限リミット回避のためのオフセット
        offset_ready = [-4, 5.09, 5.0, 5.0, 5.0, 0]

        # error reset
        print("alarm reset")
        for i in range(6):
            self.robot.alarm_reset(i)

        # 上下限リミット回避のための初期オフセット
        print("set offset")
        for i in range(6):
            self.set_offset(i, offset_ready)

        # j1, j2, j3: 速度制御モード, j4, j5, j6: 電流制御モード
        self.robot.set_control_mode([2, 2, 2, 3, 3, 3])
        time.sleep(1)

        # サーボオン
        print("servo on")
        self.robot.servo_all_on(fix_mode=False)
        time.sleep(1)

        # calibration poseへ移行
        # アームをうかせる
        print("calibration pose")
        self.robot.set_joint_reference([0, -0.2, 0.05, 0, 0, 0], relative=False)
        time.sleep(1)
        self.robot.set_joint_reference([0, 0, -0.1, 0, 0, 0], relative=False)
        time.sleep(1)

        # 軸ごとにcalib poseへ移行
        uarm_mask = [0, 0, 0, 1, 1, 1]
        print("calibration pose j2")
        self.update(1, 5, [0, -0.3, -0.2, 0, 0, 0], mask=uarm_mask)
        time.sleep(1)

        print("calibration pose j3")
        self.update(2, 5, [0, 0, -0.2, 0, 0, 0], mask=uarm_mask)
        time.sleep(1)

        print("calibration pose j1")
        self.update(0, 1.5, [0.3, 0, 0, 0, 0, 0], mask=uarm_mask)
        time.sleep(1)

        # 姿勢を固定
        self.robot.set_joint_reference([0.05, 0, 0, 0, 0, 0], relative=False)
        time.sleep(1)

        print("j1/j2/j3 calibration start")
        self.robot.calibrate_joint(0, self.calib_list[0])
        time.sleep(0.1)
        self.robot.calibrate_joint(1, self.calib_list[1])
        time.sleep(0.1)
        self.robot.calibrate_joint(2, self.calib_list[2])
        time.sleep(0.1)
        print("j1/j2/j3 calibration end")
        print("move to j4/j5/j6 calibration")

        # 位置制御モードでポーズ移動
        self.set_mode_with_retry([1, 1, 1, 1, 1, 1])
        self.robot.servo_off(3)
        self.robot.servo_off(4)
        self.robot.servo_off(5)
        self.robot.set_joint_trajectory([0.0, 1.47, 1.57, 0.0, 0.0, 0.0], 10.0, mask=[0, 0, 0, 1, 1, 1])

        print(self._make_red_text("After setting calibration pose for j4/j5, please enter Ctrl-C\n"))
        j4_angle_list = []
        j5_angle_list = []
        while True:
            angles = self.robot.get_joint_angle()
            j4_angle_list.append(angles[3])
            j5_angle_list.append(angles[4])
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                break

        memory_angle_j4 = min(j4_angle_list)
        memory_angle_j5 = min(j5_angle_list)
        input(self._make_red_text("After setting j6 to calibration pose, please enter\n"))

        print("j4/j5/j6 calibration start")
        self.robot.calibrate_joint_from_memory(3, self.calib_list[3], memory_angle_j4)
        self.robot.calibrate_joint_from_memory(4, self.calib_list[4], memory_angle_j5)
        self.robot.calibrate_joint(5, self.calib_list[5])
        print("j4/j5/j6 calibration end")

        print("servo off j4, j5, j6")
        self.robot.servo_off(3)
        self.robot.servo_off(4)
        self.robot.servo_off(5)

        input(self._make_red_text("reset to neutral pose. If it's ready, please enter\n"))

        print("servo on j4, j5, j6")
        self.robot.servo_on(3)
        self.robot.servo_on(4)
        self.robot.servo_on(5)
        time.sleep(1)

        print("move to reset pose")
        self.robot.set_joint_trajectory([0.0, 1.57, 1.57, 0.0, 0.0, 0.0], 3.0)

        print("finish auto calibration")
        self.shutdown()

    def shutdown(self):
        print("please shutdown by Ctrl-C")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.robot.servo_all_off()
                break

    def update(self, joint_no, thres_current, update_current_ref, thres_count=10, mask=None, zeroset=True):
        print("start moving")
        self.robot.set_joint_reference(update_current_ref, relative=False, mask=mask)
        cnt = 0
        while True:
            try:
                # 指定の関節の電流値が大きいと停止
                current = self.robot.get_joint_current()
                time.sleep(0.01)
                if abs(current[joint_no]) > thres_current:
                    cnt += 1

                if cnt > thres_count:
                    print("! collision joint {} !".format(joint_no + 1))
                    if zeroset:
                        self.robot.set_joint_reference([0] * 6, relative=False, mask=mask)
                    break
            except KeyboardInterrupt:
                self.shutdown()
                break
            except BaseException:
                self.shutdown()
                break
        print("stop moving")

    def set_offset(self, joint_no, offset_list):
        self.robot.calibrate_joint(joint_no, offset_list[joint_no])
        print("set offset j{}".format(joint_no + 1))
        time.sleep(0.1)

    def set_mode_with_retry(self, modes, retry_count=2):
        self.robot.set_control_mode(modes)
        time.sleep(0.5)
        for i in range(retry_count):
            if modes != self.robot.get_control_mode():
                print("retry")
                self.robot.set_control_mode(modes)
                time.sleep(2)
            else:
                break


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 ./green_modelk_auto_calibration.py 172.16.1.20")
    else:
        calibrator = AutoCalibration(sys.argv[1])
        calibrator.run()
