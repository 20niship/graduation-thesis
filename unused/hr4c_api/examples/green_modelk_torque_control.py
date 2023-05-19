import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, servo_on):
    test = GreenModelK(ipaddr)
    test.set_control_mode([4] * 6)  # 4がトルク制御

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

    # トルク制御
    print("set torque")
    torque_v = [0, -4, 0, 0, 0, 0]  # reference
    test.set_joint_reference(torque_v)
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
        print("Usage: python3 ./green_modelk_torque_control.py 172.16.1.20 True")
    else:
        print(sys.argv[1])
        main(sys.argv[1], sys.argv[2])
