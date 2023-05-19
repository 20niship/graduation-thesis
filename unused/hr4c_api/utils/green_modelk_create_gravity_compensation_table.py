#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import sys
import time
import yaml
from hr4c_api import GreenModelK


def remove_outliers(data_list):
    np_data_list = np.array(sorted(data_list))
    quartile_1, quartile_3 = np.percentile(np_data_list, [25, 75])
    iqr = quartile_3 - quartile_1
    # 下限
    lower_bound = quartile_1 - (iqr * 1.5)
    # 上限
    upper_bound = quartile_3 + (iqr * 1.5)
    return np.array(np_data_list)[((np_data_list > lower_bound) & (np_data_list < upper_bound))]


def main(ipaddr, min_j2, max_j2, min_j3, max_j3, table_tick, servo_on):
    test = GreenModelK(ipaddr)
    if servo_on == "True":
        test.servo_all_on()

    output_file = "zerog_mode_correction.yaml"
    j2_rep_time = math.floor((max_j2 - min_j2) / table_tick) + 1
    j3_rep_time = math.floor((max_j3 - min_j3) / table_tick) + 1

    # テスト初期位置に移動
    test.set_joint_trajectory([0, min_j2, min_j3, 0, 0, 0], 8.0)

    ave_time = 20
    records = {}
    measured_torques = []
    for i in range(j2_rep_time):
        measured_dict = {}
        j2_angle = min_j2 + i * table_tick
        print('j2_angle: {}'.format(j2_angle))
        measured_dict['j2_angle'] = j2_angle
        data_list = []
        for j in range(j3_rep_time):
            j3_angle = min_j3 + j * table_tick
            print('j3_angle: {}'.format(j3_angle))
            target_pose = [0, j2_angle, j3_angle, 0, 0, 0]
            test.set_joint_trajectory(target_pose, 1.0)
            time.sleep(2)
            torque_j2_list = []
            torque_j3_list = []
            for k in range(ave_time):
                torques = test.get_joint_torque()
                torque_j2_list.append(torques[1])
                torque_j3_list.append(torques[2])
                time.sleep(0.1)
            np_torque_j2_list = remove_outliers(torque_j2_list)
            np_torque_j3_list = remove_outliers(torque_j3_list)

            torque_j2 = float(np_torque_j2_list.mean())
            torque_j3 = float(np_torque_j3_list.mean())
            data_list.append({'j3_angle': j3_angle,
                              'j2_torque': torque_j2,
                              'j3_torque': torque_j3
                              })
        measured_dict['data'] = data_list
        measured_torques.append(measured_dict)
    records['torques'] = measured_torques

    # データ書き出し
    with open(output_file, "w") as f:
        yaml.dump(records, f, default_flow_style=False)

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
    if len(sys.argv) < 8:
        print("Usage: python3 ./green_modelk_create_gravity_compensation_table.py 172.16.1.20 -1.0 2.0 -1.0 2.0 5.0 True")
    else:
        main(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]),
             float(sys.argv[6]), sys.argv[7])
