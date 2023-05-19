#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
from hr4c_api import HR4CAPI


def main(ipaddr):
    test = HR4CAPI(ipaddr)

    print("calibration start")
    time.sleep(1)

    test.calibrate_joint(0, 2.26893)
    print("calibrate j1")
    time.sleep(0.1)

    test.calibrate_joint(1, -1.0472)
    print("calibrate j2")
    time.sleep(0.1)

    test.calibrate_joint(2, -0.6552)
    print("calibrate j3")
    time.sleep(0.1)

    print("calibration end")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python ./green_modelk_manual_calibration_j1j2j3.py 172.16.1.20")
    else:
        main(sys.argv[1])
