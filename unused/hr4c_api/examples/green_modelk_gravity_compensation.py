# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, servo_on):
    test = GreenModelK(ipaddr)
    test.set_control_mode([4, 4, 4, 4, 4, 4])  # 4がトルク制御
    time.sleep(2)

    # モード切り替え成否確認
    print("get_control_mode = {}".format(test.get_control_mode()))
    inp = input("right mode (4) ? :y, else: n\n")
    if inp == 'y':
        if servo_on == "True":
            test.servo_all_on(fix_mode=False)
    else:
        print("please ctrl-c to servo off !")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                test.servo_all_off()
                print("serve all off")
                return 0

    print("start")
    torque_v_old = test.get_gravity_compensation_torque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    while True:
        try:
            torque_v = test.get_gravity_compensation_torque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            if all(torque_v == [-0, -0, -0, -0, -0, -0]):
                print("! torque_v = [-0. -0. -0. -0. -0. -0.] !")
                torque_v = torque_v_old
            test.set_joint_reference(torque_v)
            torque_v_old = torque_v
            time.sleep(0.05)
        except KeyboardInterrupt:
            test.servo_all_off()
            print("servo all off")
            break
    print("finish")
    
    print("please shutdown by Ctrl-C")
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            test.servo_all_off()
            print("servo all off")
            break


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 green_modelk_gravity_compensation.py 172.16.1.20 True")
    else:
        print(sys.argv[2])
        main(sys.argv[1], sys.argv[2])
