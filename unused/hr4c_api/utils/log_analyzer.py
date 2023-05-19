#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import datetime
import numpy as np
import sys
from matplotlib import pyplot


TIME_INDEX = 0
TARGET_INDEX = 1
ANGLE_INDEX = 2
CURRENT_INDEX = 3
ALARM_INDEX = 4
LALARM_INDEX = 5
SERVO_INDEX = 6
CONTROLMODE_INDEX = 7
SPEED_INDEX = 8
TORQUE_INDEX = 9
SEQNO_INDEX = 10


def make_jointdata_list(source_logs, value_type):
    data_len = len([eval(value_type + '(' + x + ')') for x in source_logs[0].split(':') if x != ''])
    result_list = []
    for i in range(data_len):
        result_list.append([])
    for sl in source_logs:
        slist = [eval(value_type + '(' + x + ')') for x in sl.split(':') if x != '']
        for i in range(data_len):
            result_list[i].append(slist[i])

    return result_list


def plot_jointdata_figure(data_logs, value_type, title, xlabel, ylabel, yticks=None):
    pyplot.figure()
    xvalues = np.array(range(len(data_logs))) * 0.001
    yvalues = make_jointdata_list(data_logs, value_type)

    for jn, yval in enumerate(yvalues):
        pyplot.plot(xvalues, yval, label='joint' + str(jn + 1))

    pyplot.minorticks_on()
    pyplot.grid(which='major', linestyle='-', linewidth=0.3, alpha=0.6)
    pyplot.grid(which='minor', linestyle='--', linewidth=0.3, alpha=0.3)
    pyplot.title(title)
    pyplot.xlabel(xlabel)
    pyplot.ylabel(ylabel)
    if yticks is not None:
        pyplot.yticks(yticks)
    pyplot.legend()


def main(logfn):
    time_logs = []
    target_logs = []
    angle_logs = []
    current_logs = []
    alarms_logs = []
    lalarms_logs = []
    servo_logs = []
    controlmode_logs = []
    speed_logs = []
    torque_logs = []
    seqno_logs = []

    with open(logfn, 'r') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',', quotechar='"')
        for row in csv_reader:
            time_logs.append(row[TIME_INDEX])
            target_logs.append(row[TARGET_INDEX])
            angle_logs.append(row[ANGLE_INDEX])
            current_logs.append(row[CURRENT_INDEX])
            alarms_logs.append(row[ALARM_INDEX])
            lalarms_logs.append(row[LALARM_INDEX])
            servo_logs.append(row[SERVO_INDEX])
            controlmode_logs.append(row[CONTROLMODE_INDEX])
            speed_logs.append(row[SPEED_INDEX])
            torque_logs.append(row[TORQUE_INDEX])
            seqno_logs.append(row[SEQNO_INDEX])

        time_list = []
        for tl in time_logs:
            tlist = tl.split('.')
            tdate = datetime.datetime.strptime(tlist[0], '%H:%M:%S')
            tusec = int(tlist[1])
            time_list.append([tdate, tusec])
        elapse_list = []
        for i in range(len(time_list) - 1):
            usec = (time_list[i + 1][0] - time_list[i][0]).total_seconds() * 1000000
            usec += (time_list[i + 1][1] - time_list[i][1])
            elapse_list.append(usec / 1000.0)

        # 1ループごとの時間刻み
        pyplot.figure()
        xvalues = np.array(range(len(elapse_list))) * 0.001
        pyplot.plot(xvalues, elapse_list)
        pyplot.title('Elapse Time')
        pyplot.xlabel('Time(s)')
        pyplot.ylabel('Time for one loop(ms)')

        # 指令角度
        plot_jointdata_figure(target_logs, 'float', 'Target joint angle', 'Time(s)', 'Joint Angles(radian)')

        # 関節角度
        plot_jointdata_figure(angle_logs, 'float', 'Actual joint angle', 'Time(s)', 'Joint Angles(radian)')

        # 電流値
        plot_jointdata_figure(current_logs, 'float', 'Joint current', 'Time(s)', 'Joint Current(A)')

        # アラームログ
        plot_jointdata_figure(alarms_logs, 'int', 'Alarm log', 'Time(s)', 'Alarm on(1)/off(0)', yticks=[0, 1])

        # リミットアラームログ
        plot_jointdata_figure(lalarms_logs, 'int', 'Limit alarm log', 'Time(s)', 'LAlarm on(1)/off(0)', yticks=[0, 1])

        # サーボログ
        plot_jointdata_figure(servo_logs, 'int', 'Servo log', 'Time(s)', 'servo on(1)/off(0)', yticks=[0, 1])

        # 制御モード
        plot_jointdata_figure(controlmode_logs, 'int', 'Controlmode log', 'Time(s)', 'control mode',
                              yticks=[0, 1, 2, 3, 4])

        # 速度
        plot_jointdata_figure(speed_logs, 'float', 'Speed log', 'Time(s)', 'joint speed(radian/sec)')

        # トルク
        plot_jointdata_figure(torque_logs, 'float', 'Torque log', 'Time(s)', 'Torque(Nm)')

        # シーケンス番号
        pyplot.figure()
        xvalues = np.array(range(len(seqno_logs))) * 0.001
        yvalues = [int(x) for x in seqno_logs]
        pyplot.plot(xvalues, yvalues)
        pyplot.title('Seqno logs')
        pyplot.xlabel('Time(s)')
        pyplot.ylabel('Sequence no')

        pyplot.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage is: python3 ./log_analyzer.py test.log')
    else:
        main(sys.argv[1])
