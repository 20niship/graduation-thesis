#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, try_count, servo_on):
    test = GreenModelK(ipaddr)

    # 治具の取り付けチェック
    print("Please attach tool for payload test")
    input("Press enter when you are ready\n")

    # モード切替
    test.set_control_mode([1] * 6)
    print("current control_mode = {}".format(test.get_control_mode()))
    inp = input("right mode? :y, else: n\n")
    if inp == 'y':
        if servo_on == "True":
            test.servo_all_on()
    else:
        print("please shutdown by Ctrl-C")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                print("serve all off")
                test.servo_all_off()
                return 0

    upper_pose = [0.0, 0.587, 2.445, 0.0, 0.092, 0.0]
    lower_pose = [0.0, 0.810, 2.261, 0.0, 0.0, 0.0]

    # 初期位置移動
    print("goto initial pose")
    test.set_joint_trajectory(lower_pose, 5.0)

    # 負荷の取り付け
    print("Please attach test load.")
    input("Press enter when you are ready\n")

    # 上下動
    print("start test")
    for i in range(try_count):
        print("count = {}".format(i))
        test.set_joint_trajectory(upper_pose, 4.0)
        time.sleep(1)
        test.set_joint_trajectory(lower_pose, 4.0)
        time.sleep(1)

    # 負荷の取り外し
    print("Please remove test load.")
    input("Press enter when you are ready\n")

    # 終了処理
    print("please shutdown by Ctrl-C")
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("serve all off")
            test.servo_all_off()
            break


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 ./green_modelk_peyload_test.py 172.16.1.20 2 True")
    else:
        print(sys.argv[1])
        main(sys.argv[1], int(sys.argv[2]), sys.argv[3])
