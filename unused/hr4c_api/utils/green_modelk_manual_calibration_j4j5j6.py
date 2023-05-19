#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import HR4CAPI


def main(ipaddr):
    test = HR4CAPI(ipaddr)

    print("calibration start")
    time.sleep(1)

    test.calibrate_joint(3, -2.0944)
    print("calibrate j4")
    time.sleep(0.1)

    test.calibrate_joint(4, -2.1816)
    print("calibrate j5")
    time.sleep(0.1)

    test.calibrate_joint(5, 0)
    print("calibrate j6")
    time.sleep(0.1)

    print("calibration end")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python ./green_modelk_manual_calibration_j4j5j6.py 172.16.1.20")
    else:
        main(sys.argv[1])
