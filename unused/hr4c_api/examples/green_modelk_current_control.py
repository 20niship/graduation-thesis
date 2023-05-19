import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, servo_on):
    test = GreenModelK(ipaddr)
    test.set_control_mode([3] * 6)  # 3が電流制御

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

    # 電流制御
    print("set current")
    current_reference = [0, -3, 0, 0, 0, 0]  # reference
    test.set_joint_trajectory(current_reference, 3.0)
    time.sleep(3)

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
    if len(sys.argv) < 3:
        print("Usage: python3 ./green_modelk_current_control.py 172.16.1.20 True")
    else:
        print(sys.argv[1])
        main(sys.argv[1], sys.argv[2])
