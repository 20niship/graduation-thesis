# -*- coding: utf-8 -*-
import sys
from hr4c_api import HR4CAPI


def main(ipaddr, joint_no):
    test = HR4CAPI(ipaddr)
    test.alarm_reset(joint_no)
    print("alarm reset joint{}".format(joint_no))


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 ./green_modelk_alarm_reset.py 172.16.1.20 joint_no")
    else:
        main(sys.argv[1], int(sys.argv[2]))
