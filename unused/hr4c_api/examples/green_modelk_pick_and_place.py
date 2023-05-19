# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, try_count, goal_time, servo_on):
    test = GreenModelK(ipaddr)
    if servo_on == "True":
        test.servo_all_on()

    # 初期位置に移動
    test.set_joint_trajectory([0.0, 1.57, 1.57, 0.0, 0, 0.0], 3.0)

    pose1 = [0.39681497992275616, -0.17639656290299519, 0.23996438396494535]
    pose2 = [0.22374229428004563, 0.09396077213932989, 0.21869405836588027]

    print("start")
    for i in range(try_count):
        print("Count" + str(i))
        test.set_pose(*pose1, goal_time)
        test.set_pose(*pose2, goal_time)

    # 初期位置に戻る
    test.set_joint_trajectory([0.0, 1.57, 1.57, 0.0, 0, 0.0], 3.0)

    print("please shutdown by Ctrl-C")
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            test.servo_all_off()
            break


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python ./green_modelk_pick_and_place.py 172.16.1.20 10 3.0 True")
    else:
        print(sys.argv[4])
        main(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
