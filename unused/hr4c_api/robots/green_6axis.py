# -*- coding: utf-8 -*-
import bisect
import logging
import numpy as np
import math
import time
import threading
import yaml
from hr4c_api.core.hr4c_api import HR4CAPI
from hr4c_api.core.hr4c_api import hr4c_libdir
from hr4c_api.robots.hr4c_robotbase import RobotBase

logging.basicConfig()
LOGGER = logging.getLogger(__name__)
LOGGER.setLevel(level=logging.INFO)


class Green6Axis(RobotBase):
    """
    6軸マニピュレータ用抽象クラス。
    """
    def __init__(self, server_ip, h1, l1, l2, l3, min_angles, max_angles, m1, m2, m3, m4, m5, m6):
        """
        初期化メソッド
        :param server_ip: コントローラBOX内制御計算機のIPアドレス
        :param h1:  　モデル上の原点からJ2までのリンク長。単位はm
        :param l1:  　モデル上のJ2からJ3までのリンク長。単位はm
        :param l2:  　モデル上のJ3からJ5までのリンク長。単位はm
        :param l3:  　モデル上のJ5からTCP(ToolCenterPoint)までのリンク長。単位はm
        :param min_angles: 　最小角度列。単位はradian
        :param max_angles: 　最大角度列。単位はradian
        :param m1:     リンク1質量。単位はkg
        :param m2:     リンク2質量。単位はkg
        :param m3:     リンク3質量。単位はkg
        :param m4:     リンク4質量。単位はkg
        :param m5:     リンク5質量。単位はkg
        :param m6:     リンク6質量。単位はkg
        """
        # 親クラスの初期化
        super().__init__(server_ip, "green", 6, h1, l1, l2, min_angles=min_angles, max_angles=max_angles)

        # 6軸固有のパラメータ
        self._l3 = l3
        self._x4_v = self._x3_v
        self._y4_v = self._y3_v
        self._z4_v = self._z3_v
        self._x5_v = self._x4_v
        self._y5_v = self._y4_v
        self._z5_v = self._z4_v
        self._x6_v = self._x5_v
        self._y6_v = self._y5_v
        self._z6_v = self._z5_v

        self._p5_v = self._p4_v
        self._p6_v = self._p3_v + self._l3 * self._z5_v
        self._zerog_mode = False
        self._zerog_thr = None
        self._correction_dict = None

        # mass parameterのセット
        self._set_mass_parameter(m1, m2, m3, m4, m5, m6)

        # デーモンスレッドのスタート
        daemon_thr = threading.Thread(target=self._monitor_zerog_mode_thread)
        daemon_thr.daemon = True
        daemon_thr.start()

    def _monitor_zerog_mode_thread(self):
        main_thread = threading.main_thread()
        main_thread.join()
        if self._zerog_mode:
            LOGGER.warning('Try to stop zerog mode')
        self._stop_zerog_mode()

    def update_larm_positions(self, j4, j5, j6):
        """
        前腕ユニットのモデル位置を更新する
        :param j4: J4の関節角度。単位はradian
        :param j5: J5の関節角度。単位はradian
        :param j6: J6の関節角度。単位はradian
        :return: なし
        """
        # update orientation unit vector
        self._x4_v = math.cos(j4) * self._x3_v + math.sin(j4) * self._y3_v
        self._y4_v = -1 * math.sin(j4) * self._x3_v + math.cos(j4) * self._y3_v
        self._z4_v = self._z3_v

        self._x5_v = math.cos(j5) * self._x4_v - math.sin(j5) * self._z4_v
        self._y5_v = self._y4_v
        self._z5_v = math.sin(j5) * self._x4_v + math.cos(j5) * self._z4_v

        self._x6_v = math.cos(j6) * self._x5_v + math.sin(j6) * self._y5_v
        self._y6_v = -1 * math.sin(j6) * self._x5_v + math.cos(j6) * self._y5_v
        self._z6_v = self._z5_v

        # update position of TCP
        self._p5_v = self._p4_v
        self._p6_v = self._p5_v + self._l3 * self._z5_v

    def calc_actual_angles(self, joints):
        """
        モデル上の角度から実際のモータ角度への変換を行う
        :param joints: モデル上の関節角度列。単位はradian
        :return:　実際の関節角度列。単位はradian
        """
        j1 = joints[0]
        j2 = joints[1]
        j3 = joints[2]
        actual_j3 = j3 + j2
        j4 = joints[3]
        j5 = joints[4]
        j6 = joints[5]
        return [j1, j2, actual_j3, j4, j5, j6]

    def calc_model_angles(self, joints):
        """
        実際のモータ角度からモデル上の角度への変換を行う
        :param joints:　実際の関節角度列。単位はradian
        :return:　モデル上の関節角度列。単位はradian
        """
        j1 = joints[0]
        j2 = joints[1]
        j3 = joints[2]
        model_j3 = j3 - j2
        j4 = joints[3]
        j5 = joints[4]
        j6 = joints[5]
        return [j1, j2, model_j3, j4, j5, j6]

    def calc_FK(self, joints):
        """
        FKを解くメソッド
        :param joints: モデル上の関節角度列
        :return:　デカルト座標系での位置（x, y, z)。単位はm
        """
        self.update_uarm_positions(*joints[0:3])
        self.update_larm_positions(*joints[3:6])
        x = self._p6_v[0]
        y = self._p6_v[1]
        z = self._p6_v[2]
        return x, y, z

    def calc_IK(self, x, y, z):
        """
        IKを解くメソッド。デカルト座標系でX軸プラス方向がロボット正面を向く右手座標系で指定。
        一般的な位置・姿勢のIKではなく、姿勢については手先が鉛直方向を向いている特別な状態でのIKを解く
        :param x:　X座標値。単位はm
        :param y:　Y座標値。単位はm
        :param z:　Z座標値。単位はm
        :return:　順に、IKが成功したか、J1、J2、J3、J4、J5、J6の角度（単位はradian)
        """
        tx = x
        ty = y
        tz = z + self._l3
        ret, j1, j2, j3 = self.calc_uarm_IK(tx, ty, tz)

        if ret:
            # update uarm positions
            self.update_uarm_positions(j1, j2, j3)

            j4 = 0.0
            uz = np.array([0.0, 0.0, -1.0])
            cos_j5 = np.dot(self._z3_v, uz) / (np.linalg.norm(self._z3_v) * np.linalg.norm(uz))
            sin_j5 = math.sqrt(1.0 - cos_j5 ** 2)
            j5 = math.atan2(sin_j5, cos_j5)
            j6 = 0.0
        else:
            j4 = 0.0
            j5 = 0.0
            j6 = 0.0

        return ret, j1, j2, j3, j4, j5, j6

    def set_pose(self, x, y, z, goal_time, relative=False,
                 interpolation_method=HR4CAPI.MINJERK, wait_interpolation=True):
        """
        手先のデカルト座標系での位置を指定して補間移動させる
        :param x:　目標のX座標値。単位はm
        :param y:　目標のY座標値。単位はm
        :param z:　目標のZ座標値。単位はm
        :param goal_time:　目標到達時間。単位はs
        :param relative:　相対値かどうかのフラグ。デフォルトはFalse
        :param interpolation_method:　補間方法。デフォルトはMINJERK補間。
        :param wait_interpolation: 補間終了まで待機するかどうか。デフォルトはTrue
        :return:　なし
        """
        control_modes = self.get_control_mode()
        if not all([cm == HR4CAPI.CONTROLMODE_POSITION for cm in control_modes]):
            LOGGER.warning('All joints should be controlled by position mode for using this method!')
            return

        if relative:
            cur_x, cur_y, cur_z = self.get_uarm_pose()
            tgt_x = cur_x + x
            tgt_y = cur_y + y
            tgt_z = cur_z + z
        else:
            tgt_x = x
            tgt_y = y
            tgt_z = z

        ret, j1, j2, j3, j4, j5, j6 = self.calc_IK(tgt_x, tgt_y, tgt_z)
        if ret:
            self.set_joint_trajectory([j1, j2, j3, j4, j5, j6], goal_time, None, False, interpolation_method,wait_interpolation=wait_interpolation)

    def get_pose(self):
        """
        手先の現在のデカルト座標系での位置を取得する
        :return:　デカルト座標系での位置（x, y, z)。単位はm
        """
        joint_angles = self.get_joint_angle()
        return self.calc_FK(joint_angles)

    def set_force_moment(self, fx, fy, fz, mx, my, mz, mask=(0, 0, 0, 0, 0, 0)):
        """
        指定した手先の力、モーメントを発生する各関節トルクを目標としてセットするメソッド。トルク制御モード専用。
        :param fx:　X軸方向の力。単位はN
        :param fy:　Y軸方向の力。単位はN
        :param fz:　Z軸方向の力。単位はN
        :param mx:　X軸周りのモーメント。単位はNm
        :param my:　Y軸周りのモーメント。単位はNm
        :param mz:　Z軸周りのモーメント。単位はNm
        :param mask: 関節マスク, 1：マスクあり、0: マスクなし
        :return:　なし
        """
        control_modes = self.get_control_mode()
        if not all([cm == HR4CAPI.CONTROLMODE_TORQUE for cm in np.array(control_modes)[(1-np.array(mask)) > 0]]):
            LOGGER.warning('All joints should be controlled by torque mode for using this method!')
            return

        joint_angles = self.get_joint_angle()
        self.update_uarm_positions(*joint_angles[0:3])
        self.update_larm_positions(*joint_angles[3:6])

        # statics (calc. of torques for generating force vector)
        # force vector(N)
        force_v = np.array([fx, fy, fz])

        # moment vector(Nm)
        moment_v = np.array([mx, my, mz])

        # joint torque(Nm)
        torque1 = np.dot(np.cross(self._z1_v, self._p6_v - self._p1_v), force_v) + np.dot(self._z1_v, moment_v)
        torque2 = np.dot(np.cross(self._y2_v, self._p6_v - self._p2_v), force_v) + np.dot(self._y2_v, moment_v)
        torque3 = np.dot(np.cross(self._y3_v, self._p6_v - self._p3_v), force_v) + np.dot(self._y3_v, moment_v)
        torque4 = np.dot(np.cross(self._z4_v, self._p6_v - self._p4_v), force_v) + np.dot(self._z4_v, moment_v)
        torque5 = np.dot(np.cross(self._y5_v, self._p6_v - self._p5_v), force_v) + np.dot(self._y5_v, moment_v)
        torque6 = np.dot(self._z6_v, moment_v)

        # send joint reference
        LOGGER.debug("torque: ({0}, {1}, {2}, {3}, {4}, {5})".format(torque1, torque2, torque3, torque4, torque5, torque6))

        torque_array = np.array([torque1, torque2, torque3, torque4, torque5, torque6]) * (1 - np.array(mask))
        self.set_joint_reference(torque_array.tolist(), mask=mask)
        return [torque1, torque2, torque3, torque4, torque5, torque6]

    def get_force_moment(self, torque1, torque2, torque3, torque4, torque5, torque6):
        """
        各軸の関節トルクから手先に発生する力、モーメントを計算するメソッド。
        :param torque1:　J1軸のトルク。単位はNm
        :param torque2:　J2軸のトルク。単位はNm
        :param torque3:　J3軸のトルク。単位はNm
        :param torque4:　J4軸のトルク。単位はNm
        :param torque5:　J5軸のトルク。単位はNm
        :param torque6:　J6軸のトルク。単位はNm
        :return:　(fx, fy, fz, mx, my, mz)を返す。fx, fy, fzは手先力（単位N)、mx, my, mzは手先モーメント（単位はNm)
        """
        joint_angles = self.get_joint_angle()
        self.update_uarm_positions(*joint_angles[0:3])
        self.update_larm_positions(*joint_angles[3:6])
        mat = np.array([np.concatenate([np.cross(self._z1_v, self._p6_v - self._p1_v), self._z1_v]),
                        np.concatenate([np.cross(self._y2_v, self._p6_v - self._p2_v), self._y2_v]),
                        np.concatenate([np.cross(self._y3_v, self._p6_v - self._p3_v), self._y3_v]),
                        np.concatenate([np.cross(self._z4_v, self._p6_v - self._p4_v), self._z4_v]),
                        np.concatenate([np.cross(self._y5_v, self._p6_v - self._p5_v), self._y5_v]),
                        np.concatenate([np.cross(self._z6_v, self._p6_v - self._p6_v), self._z6_v])])
        inv_mat = np.linalg.inv(mat)
        torque_v = np.array([torque1, torque2, torque3, torque4, torque5, torque6])
        force_moment_array = np.dot(inv_mat, torque_v)
        return force_moment_array

    def detect_collision(self, goal_references, threshold_check_finish_motion, threshold_current,
                         force_stop=True, target=None):
        """
        1フレーム前の電流値と現在の電流値の差分が閾値以上だと環境と接触したと判断してforce_stop()で強制停止させるメソッド。
        :param goal_references: 関節制御目標値
        :param threshold_check_finish_motion: 動作終了判定用の関節制御閾値。単位はradian
        :param threshold_current:  衝突検知用の電流閾値。単位はA
        :param force_stop: 強制停止をするかどうか(Falseの場合は返り値のみ返す)。デフォルトはTrue
        :param target: 補完終了判定で注目する状態値(position, angle, speed, current, torque)
        :return: True (collision) or False (non collision)
        """
        while not self.check_finish_motion(goal_references, threshold_check_finish_motion, target):
            collision_detection_list = abs(np.array(self.get_joint_current())) >= threshold_current
            time.sleep(0.01)
            if any(collision_detection_list):
                collision_jt = np.where(collision_detection_list)[0][0]
                LOGGER.info("! collision joint {} !".format(collision_jt))

                if force_stop:
                    self.force_stop()
                    LOGGER.info("force stop !")
                return True
        return False

    def check_finish_motion(self, goal_references, threshold, target=None):
        """
        動作終了判定(現在の関節値と関節制御目標値の差分が閾値以下（補完終了）だとTrue, 閾値以上（補完未終了）だとFalseを返す)メソッド。
        :param goal_references:　関節制御目標値。単位はradian
        :param threshold: 　動作終了判定用の関節制御閾値。単位はradian
        :param target: 注目する状態値(position, speed, current, torque) Noneの場合はコントロールモードを取得して使用
        :return: True or False
        """
        if target :
            current_state = eval("self.get_joint_{}()".format(target))
        else:
            current_control_modes = self.get_control_mode()
            angles = self.get_joint_angle()
            speeds = self.get_joint_speed()
            currents = self.get_joint_current()
            torques = self.get_joint_torque()
            current_state = []
            for i, cm in enumerate(current_control_modes):
                if cm == HR4CAPI.CONTROLMODE_POSITION:
                    current_state.append(angles[i])
                elif cm == HR4CAPI.CONTROLMODE_SPEED:
                    current_state.append(speeds[i])
                elif cm == HR4CAPI.CONTROLMODE_CURRENT:
                    current_state.append(currents[i])
                elif cm == HR4CAPI.CONTROLMODE_TORQUE:
                    current_state.append(torques[i])
                else:
                    current_state.append(0.0)
        current_state_np = np.array(current_state)
        goal_references_np = np.array(goal_references)
        return all([abs(item) <= threshold for item in (goal_references_np - current_state_np)])

    def get_gravity_compensation_torque(self, torque1, torque2, torque3, torque4, torque5, torque6):
        """
        重力補償トルクを求めるメソッド。
        :param torque1: 　J1の目標トルク。単位はNm
        :param torque2: 　J2の目標トルク。単位はNm
        :param torque3: 　J3の目標トルク。単位はNm
        :param torque4: 　J4の目標トルク。単位はNm
        :param torque5: 　J5の目標トルク。単位はNm
        :param torque6: 　J6の目標トルク。単位はNm
        :return: 重力補償されたトルク列。
        """
        joint_angles = self.get_joint_angle()
        self.update_uarm_positions(*joint_angles[0:3])
        self.update_larm_positions(*joint_angles[3:6])

        # mass of links
        if self.get_joint_angle()[1] < math.pi / 5.0:
            self._s2 = np.array([-0.0333, 0, 0.0558])
        else:
            self._s2 = np.array([-0.0333, 0, self._l1 / 2.0])

        M = np.array([self._m1, self._m2, self._m3, self._m4, self._m5, self._m6], dtype="float64").reshape([6, 1])

        # distance between joints
        s_var_array = np.array([np.concatenate([self._s1, [1]]).reshape([4,1]),
                                np.concatenate([self._s2, [1]]).reshape([4,1]),
                                np.concatenate([self._s3, [1]]).reshape([4,1]),
                                np.concatenate([self._s4, [1]]).reshape([4,1]),
                                np.concatenate([self._s5, [1]]).reshape([4,1]),
                                np.concatenate([self._s6, [1]]).reshape([4,1])])

        T = []
        dT = []
        for i in range(6):
            T.append(self._calc_T(self._axis[i], joint_angles[i], i))
            dT.append(self._calc_dT(T[i], self._axis[i]))

        dT_dq = np.array([np.array([dT[0],            dT[0]@T[1],        dT[0]@T[1]@T[2],  dT[0]@T[1]@T[2]@T[3], dT[0]@T[1]@T[2]@T[3]@T[4], dT[0]@T[1]@T[2]@T[3]@T[4]@T[5]], dtype="float64"),
                          np.array([np.zeros([4, 4]), T[0]@dT[1],        T[0]@dT[1]@T[2],  T[0]@dT[1]@T[2]@T[3], T[0]@dT[1]@T[2]@T[3]@T[4], T[0]@dT[1]@T[2]@T[3]@T[4]@T[5]], dtype="float64"),
                          np.array([np.zeros([4, 4]), np.zeros([4, 4]),  T[0]@T[1]@dT[2],  T[0]@T[1]@dT[2]@T[3], T[0]@T[1]@dT[2]@T[3]@T[4], T[0]@T[1]@dT[2]@T[3]@T[4]@T[5]], dtype="float64"),
                          np.array([np.zeros([4, 4]), np.zeros([4, 4]),  np.zeros([4, 4]), T[0]@T[1]@T[2]@dT[3], T[0]@T[1]@T[2]@dT[3]@T[4], T[0]@T[1]@T[2]@dT[3]@T[4]@T[5]], dtype="float64"),
                          np.array([np.zeros([4, 4]), np.zeros([4, 4]),  np.zeros([4, 4]), np.zeros([4, 4]),     T[0]@T[1]@T[2]@T[3]@dT[4], T[0]@T[1]@T[2]@T[3]@dT[4]@T[5]], dtype="float64"),
                          np.array([np.zeros([4, 4]), np.zeros([4, 4]),  np.zeros([4, 4]), np.zeros([4, 4]),     np.zeros([4, 4]),          T[0]@T[1]@T[2]@T[3]@T[4]@dT[5]], dtype="float64")],
                         dtype="float64")

        g_var = np.array([0, 0, -9.8, 0]).reshape([4, 1])
        G = - ((g_var.T @ dT_dq @ s_var_array).reshape([6, 6]) @ M).reshape([6])
        # J2, J3補正ファイルを使う場合
        if self._correction_dict is not None:
            G_torque2, G_torque3 = self._calc_compensation_torques_from_table(joint_angles[1], joint_angles[2])
            if G_torque2 is not None:
                G[1] = G_torque2
            if G_torque3 is not None:
                G[2] = G_torque3

        LOGGER.debug("G = {}".format(G))
        torque_v = np.array([torque1, torque2, torque3, torque4, torque5, torque6])
        torque_v += G

        return torque_v

    def set_zerog_mode(self, set_flag, interval=0.01, correction_file="zerog_mode_correction.yaml"):
        """
        重力補償され、軽く動かせる状態に変更するメソッド。
        :param set_flag: Trueの場合にzero-Gモードに変更。Falseでzero-Gモードを終了する。
        :param interval: ゼロGの関節トルクを更新する周期。デフォルトは0.05秒。
        :param correction_file: J2/J3軸補正用のファイル名。デフォルトはzerog_mode_correction.yaml。Noneを指定すると計算値を使う
        :return:　なし
        """
        if set_flag:
            if not self._zerog_mode:
                control_modes = self.get_control_mode()
                if not all([cm == HR4CAPI.CONTROLMODE_TORQUE for cm in control_modes]):
                    LOGGER.warning('All joints should be controlled by torque mode for using this method!')
                    return
                self._zerog_mode = True
                if correction_file is not None:
                    self._correction_dict = self._load_correction_file(correction_file)
                else:
                    self._correction_dict = None
                self._zerog_thr = threading.Thread(target=self._set_zerog_torque_loop, args=(interval, ))
                self._zerog_thr.start()
            else:
                LOGGER.warning('Already in zero-G mode!')
        else:
            if not self._zerog_mode:
                LOGGER.warning('Not in zero-G mode!')
            self._stop_zerog_mode()

    def _stop_zerog_mode(self):
        if self._zerog_mode:
            self._zerog_mode = False
            self._zerog_thr.join()
            self._zerog_thr = None

    def _load_correction_file(self, correction_file):
        correction_file_path = hr4c_libdir + "/" + correction_file

        correction_data = None
        try:
            with open(correction_file_path) as f:
                correction_data = yaml.load(f, Loader=yaml.SafeLoader)
        except OSError as err:
            LOGGER.error('OS error: {0}'.format(err))

        return correction_data

    def _calc_compensation_torques_from_table(self, j2_angle, j3_angle):
        # 記録されている範囲外の場合はNoneを返す
        torque2 = None
        torque3 = None

        if self._correction_dict is not None:
            j2_tor_data = sorted(self._correction_dict['torques'], key=lambda x: float(x['j2_angle']))
            j2_angle_data = [float(x['j2_angle']) for x in j2_tor_data]
            if j2_angle < j2_angle_data[0] or j2_angle > j2_angle_data[-1]:
                # J2について、記録されている範囲外の場合はNoneを返す
                LOGGER.warning('Out of recorded joint2 range!: {}'.format(j2_angle))
                return torque2, torque3
            else:
                j2_index = bisect.bisect_left(j2_angle_data, j2_angle)
                j3_tor_data = sorted(j2_tor_data[j2_index]['data'], key=lambda x: float(x['j3_angle']))
                j3_angle_data = [float(x['j3_angle']) for x in j3_tor_data]
                if j3_angle <= j3_angle_data[0] or j3_angle >= j3_angle_data[-1]:
                    # J3について、記録されている範囲外の場合は外挿データを返す
                    if j3_angle <= j3_angle_data[0]:
                        prev_index = 0
                        following_index = 1
                    else:
                        prev_index = -1
                        following_index = -2
                else:
                    # 記録されている値を使い補間計算
                    j3_index = bisect.bisect_left(j3_angle_data, j3_angle)
                    prev_index = j3_index - 1
                    following_index = j3_index

                # 補間計算
                if prev_index is not None and following_index is not None:
                    j3_previous_ang = float(j3_tor_data[prev_index]['j3_angle'])
                    j3_following_ang = float(j3_tor_data[following_index]['j3_angle'])
                    j2_previous_tor = float(j3_tor_data[prev_index]['j2_torque'])
                    j2_following_tor = float(j3_tor_data[following_index]['j2_torque'])
                    j3_previous_tor = float(j3_tor_data[prev_index]['j3_torque'])
                    j3_following_tor = float(j3_tor_data[following_index]['j3_torque'])
                    interpolate_coef = (j3_angle - j3_previous_ang) / (j3_following_ang - j3_previous_ang)
                    torque2 = interpolate_coef * (j2_following_tor - j2_previous_tor) + j2_previous_tor
                    torque3 = interpolate_coef * (j3_following_tor - j3_previous_tor) + j3_previous_tor

                return torque2, torque3

        return torque2, torque3

    def _set_zerog_torque_loop(self, interval):
        torque_v_old = self.get_gravity_compensation_torque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        while self._zerog_mode:
            torque_v = self.get_gravity_compensation_torque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            if all(torque_v == [-0, -0, -0, -0, -0, -0]):
                LOGGER.warning('! torque_v = [-0. -0. -0. -0. -0. -0.] !')
                torque_v = torque_v_old

            self.set_joint_reference(torque_v)
            torque_v_old = torque_v
            time.sleep(interval)

    def _set_mass_parameter(self, m1, m2, m3, m4, m5, m6):
        # mass param
        self._m1 = m1
        self._m2 = m2
        self._m3 = m3
        self._m4 = m4
        self._m5 = m5
        self._m6 = m6

        # distance between joints
        self.distance_between_joints = [[0, 0, 0], [0, 0, self._h1], [0, 0, self._l1], [0, 0, self._l2],
                                        [0, 0, 0], [0, 0, self._l3]]

        # pos of center of mass
        self._s1 = np.array([0, 0, -self._h1 / 2.0])
        self._s2 = np.array([-0.0333, 0, 0.0558])
        self._s3 = np.array([0, 0, 3.0 * self._l2 / 5.0])
        self._s4 = np.array([0] * 3)
        self._s5 = np.array([0, 0, self._l3 / 3.0])
        self._s6 = np.array([0] * 3)

        self._axis = ["z", "y", "y", "z", "y", "z"]

    def _calc_T(self, axis, theta, joint_no):
        if axis == "x":
            T = np.block([np.vstack([np.vstack([[1, 0, 0],
                                                [0, math.cos(theta), - math.sin(theta)],
                                                [0, math.sin(theta),
                                                 math.cos(theta)]]),
                                     np.array([0] * 3)]),
                          np.concatenate([self.distance_between_joints[joint_no], [1]]).reshape([4, 1])])
        elif axis == "y":
            T = np.block([np.vstack([np.vstack([[math.cos(theta),
                                                 0,
                                                 math.sin(theta)],
                                                [0, 1, 0],
                                                [-math.sin(theta),
                                                 0,
                                                 math.cos(theta)]]),
                                     np.array([0] * 3)]),
                          np.concatenate([self.distance_between_joints[joint_no], [1]]).reshape([4, 1])])
        elif axis == "z":
            T = np.block([np.vstack([np.vstack([[math.cos(theta), -math.sin(theta), 0],
                                                [math.sin(theta), math.cos(theta), 0],
                                                [0, 0, 1]]),
                                     np.array([0] * 3)]),
                          np.concatenate([self.distance_between_joints[joint_no], [1]]).reshape([4, 1])])
        return T

    def _calc_dT(self, T, axis):
        lam_dict = {"x": np.array([[0, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 0]], dtype="float64"),
                    "y": np.array([[0, 0, 1, 0], [0, 0, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 0]], dtype="float64"),
                    "z": np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]], dtype="float64")}
        dT = T @ lam_dict[axis]
        return dT
