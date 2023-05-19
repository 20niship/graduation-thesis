import sys
import time
from hr4c_api import GreenModelK
import numpy as np


def main(ipaddr, try_count, servo_on):
    test = GreenModelK(ipaddr)
    test.set_control_mode([2] * 6)  # 2が速度制御

    # モード切り替え成否確認
    print("current control_mode = {}".format(test.get_control_mode()))
    inp = input("right mode? :y, else: n\n")
    if inp == 'y':
        if servo_on == "True":
            # 全軸同時にサーボオンする形で良いためfix_mode=Falseにする
            test.servo_all_on(fix_mode=False)
    else:
        print("please shutdown by Ctrl-C")
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                print("serve all off")
                test.servo_all_off()
                return

    # 速度制御
    print("set current")
    speed_ref = np.array([0, -0.1, 0, 0, 0, 0])
    for i in range(try_count):
        print("count = {}".format(i))
        test.set_joint_reference((-1)**i * speed_ref, relative=False)
        time.sleep(3)
        
    print("finish")

    test.set_joint_reference([0] * 6, relative=False)

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
        print("Usage: python3 ./green_modelk_speed_control.py 172.16.1.20 2 True")
    else:
        print(sys.argv[1])
        main(sys.argv[1], int(sys.argv[2]), sys.argv[3])
