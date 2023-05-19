# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import GreenModelK


def main(ipaddr, mode_no):
    test = GreenModelK(ipaddr)
    print("[mode_num]*6 = {}".format([mode_no]*6))
    test.set_control_mode([mode_no]*6)
    time.sleep(1)
    print("get_control_mode = {}".format(test.get_control_mode()))


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 ./green_modelk_set_control_mode.py 172.16.1.20 mode_no")
    else:
        print("sys.argv[1] = {}".format(sys.argv[1]))
        main(sys.argv[1], int(sys.argv[2]))
