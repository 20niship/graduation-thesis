import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, try_count, servo_on):
    test = GreenModelK(ipaddr)
    test.set_control_mode([1] * 6)  # 1が位置制御

    # モード切り替え成否確認
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

    # 位置制御
    print("set pose")
    pose_references = [[0.0, 1.57, 1.57, 0.0, 0, 0.0],
                       [0.0, 0.78, 1.57, 0.0, 0, 0.0]]
    
    test.set_joint_trajectory(pose_references[0], 5.0)
    
    for i in range(try_count):
        print("count = {}".format(i))
        test.set_joint_trajectory(pose_references[1], 4.0)
        time.sleep(1)
        test.set_joint_trajectory(pose_references[0], 4.0)
        time.sleep(1)

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
        print("Usage: python3 ./green_modelk_position_control.py 172.16.1.20 2 True")
    else:
        print(sys.argv[1])
        main(sys.argv[1], int(sys.argv[2]), sys.argv[3])
