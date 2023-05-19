#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from hr4c_api.core.hr4c_api import HR4CAPI


def main(ipaddr, bin_dir):
    test = HR4CAPI(ipaddr)
    print("update controller software")
    test.update_controller_software(bin_dir)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python ./update_controller_binary.py 172.16.1.20 ./")
    else:
        main(sys.argv[1], sys.argv[2])
