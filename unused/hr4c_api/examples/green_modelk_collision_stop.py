# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, try_count, goal_time, servo_on):
    test = GreenModelK(ipaddr)
    test.set_control_mode([1] * 6)  # 1が位置制御

    # モード切り替え成否確認
    print("current control_mode = {}".format(test.get_control_mode()))
    inp = input("right mode? :y, else: n\n")
    if inp == 'y':
        if servo_on == "True":
            test.servo_all_on(fix_mode=True)
    else:
        print("please shutdown by Ctrl-C")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                print("serve all off")
                test.servo_all_off()
                return

    test.set_joint_trajectory([0.0, 1.47, 1.57, 0.0, 0, 0.0], 4.0)

    traj1 = [0.0, 0.52, 1.57, 0.0, 0, 0.0]
    traj2 = [0.0, 1.57, 1.57, 0.0, 0, 0.0]
    threshold_angle = 0.1
    threshold_current = [4] * 6

    print("start")
    for i in range(try_count):
        print("Count" + str(i))
        test.set_joint_trajectory(traj1, goal_time, wait_interpolation=False)
        test.detect_collision(traj1, threshold_angle, threshold_current, force_stop=True)
        time.sleep(3.0)

        test.set_joint_trajectory(traj2, goal_time, wait_interpolation=False)
        test.detect_collision(traj2, threshold_angle, threshold_current, force_stop=True)
        time.sleep(3.0)
    print("finish")

    print("please shutdown by Ctrl-C")
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            test.servo_all_off()
            break


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("Usage: python3 ./green_modelk_collision_stop.py 172.16.1.20 3 8.0 True")
    else:
        print(sys.argv[4])
        main(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
