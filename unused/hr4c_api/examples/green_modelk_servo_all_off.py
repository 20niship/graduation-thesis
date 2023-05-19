# -*- coding: utf-8 -*-
import sys
from hr4c_api import GreenModelK
import time


def main(ipaddr):
    test = GreenModelK(ipaddr)
    time.sleep(1)
    print("please shutdown by Ctrl-C to servo off")
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            test.servo_all_off()
            print("servo_all_off")
            break


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 ./green_modelk_servo_all_off.py 172.16.1.20")
    else:
        main(sys.argv[1])
