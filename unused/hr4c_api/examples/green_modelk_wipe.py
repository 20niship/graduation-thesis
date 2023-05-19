# -*- coding: utf-8 -*-
import sys
import time
import math
import numpy as np
from hr4c_api import GreenModelK


class Wipe(object):
    # 力制御で平面に手先を沿わせた状態で目標位置と現在位置の間を往復する動作デモ
    def __init__(self, ipaddr, try_count, servo_on):
        self.robot = GreenModelK(ipaddr)

        # パラメタの定義
        self.scale = 7.5
        self.update_scale = 0.01
        self.threshold_check_finish_motion_norm = 0.04
        theta = - math.pi / 5.0

        pose1 = [0.2589217093665835, -0.005309049539318588, 0.1536051881640445]
        pose2 = [0.25969980915322805, 0.15643109121655607, 0.00811383981557101]

        # 位置制御
        self.mode_set([1]*6, servo_on, fix_mode=True)
        self.robot.set_pose(*pose1, 5.0)
        self.robot.set_pose(*pose2, 5.0)

        print("shutdown once by Ctrl-C and set the head on the ground")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.robot.servo_all_off()
                print("servo all off")
                break

        # goal_posを決定
        r = np.linalg.norm(self.robot.get_pose()[:2])
        angle = self.robot.get_joint_angle()
        goal_pos = np.array([r * math.cos(angle[0] + theta), r * math.sin(angle[0] + theta), self.robot.get_pose()[2]])
        print("current_pos = {}: goal_pos = {}".format(self.robot.get_pose(), goal_pos))

        # j1,j2,j3はトルク制御、j4、j5、j6は位置制御
        self.mode_set([4, 4, 4, 1, 1, 1], servo_on, fix_mode=False)
        self.robot.set_force_moment(*([0, 0, 0.1, 0, 0, 0]), mask=[0, 0, 0, 1, 1, 1])

        reference_force_moment_array = np.array([0, 0, 6, 0, 0, 0])
        print("reference_force_moment_array = {}".format(reference_force_moment_array))

        # 力の発揮、executed_torque_arrayは関節トルク列
        executed_torque_array = self.robot.set_force_moment(*reference_force_moment_array, mask=[0, 0, 0, 1, 1, 1])

        # 面に接触した際の関節トルクを手先の力、モーメントに変換
        const_force_moment_array = np.array(self.robot.get_force_moment(*executed_torque_array))
        time.sleep(3.0)

        # 目標位置と動作開始位置のリスト
        pos_list = [goal_pos, np.array(self.robot.get_pose())]

        input("start wipe? :y, else: n\n")

        # 目標位置と動作開始位置の間の反復動作
        for i in range(try_count):
            print("Count" + str(i))

            # 目標位置と現在位置の差分から移動ベクトルを計算
            move_direction_2d = (pos_list[i % 2] - np.array(self.robot.get_pose()))[:2] / np.linalg.norm((pos_list[i % 2] - np.array(self.robot.get_pose()))[:2])

            # 初期押し付け力に移動ベクトルの定数倍を加えて目標発揮力を決定
            wipe_force_moment_array = np.concatenate([self.scale * move_direction_2d, np.array([0]*4)]) + const_force_moment_array
            print("wipe_force_moment_array = {}".format(wipe_force_moment_array))

            # 目標力を発揮
            self.robot.set_force_moment(*wipe_force_moment_array, mask=[0, 0, 0, 1, 1, 1])

            # 目標位値と現在位置のx,y方向の差分ベクトル
            diff_pos_2d = (pos_list[i % 2] - np.array(self.robot.get_pose()))[:2]

            # 目標位置への到達判定、到達すると抜ける
            while np.linalg.norm(diff_pos_2d) >= self.threshold_check_finish_motion_norm:
                try:
                    diff_pos_2d = (pos_list[i % 2] - np.array(self.robot.get_pose()))[:2]
                    time.sleep(0.001)
                except KeyboardInterrupt:
                    print("exit from loop")
                    break

            # 目標位置へ到達すると初期押し付け力を発揮
            self.robot.set_force_moment(*reference_force_moment_array, mask=[0, 0, 0, 1, 1, 1])
            time.sleep(2)

        print("finish")
        print("please shutdown by Ctrl-C")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.robot.servo_all_off()
                print("servo all off")
                break
        return

    def mode_set(self, mode, servo_on, fix_mode=True):
        self.robot.set_control_mode(mode)
        time.sleep(1)
        # モード切り替え成否確認
        current_control_mode = self.robot.get_control_mode()
        if mode != current_control_mode:
            print("retry")
            self.robot.set_control_mode(mode)
            time.sleep(2)
            current_control_mode = self.robot.get_control_mode()

        print("current control_mode = {}".format(current_control_mode))
        while input("right mode? :y, else: n\n") != 'y':
            try:
                print("retry")
                self.robot.set_control_mode(mode)
                time.sleep(1)
                print(self.robot.get_control_mode())
            except KeyboardInterrupt:
                print("serve all off")
                self.robot.servo_all_off()
                return 0

        if servo_on == "True":
            self.robot.servo_all_on(fix_mode=fix_mode, joint_list=[0, 1, 2, 3, 4])  # j6はoffのまま


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 ./green_modelk_wipe.py 172.16.1.20 2 True")
    else:
        print(sys.argv[1])
        instance = Wipe(sys.argv[1], int(sys.argv[2]), sys.argv[3])
