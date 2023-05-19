# -*- coding: utf-8 -*-
import logging
import math
import numpy as np
from abc import ABCMeta, abstractmethod
from hr4c_api.core.hr4c_api import HR4CAPI

logging.basicConfig()
LOGGER = logging.getLogger(__name__)
LOGGER.setLevel(level=logging.INFO)


class RobotBase(HR4CAPI, metaclass=ABCMeta):
    """
    ３軸上腕ユニットを備えたロボットの共通抽象クラス。
    """
    def __init__(self, server_ip, model, dof, h1, l1, l2, min_angles=None, max_angles=None):
        """
        初期化メソッド
        :param server_ip: コントローラBOX内制御計算機のIPアドレス
        :param model: ロボットモデル名。
        :param dof: 自由度数
        :param h1: 　モデル上の原点からJ2までのリンク長。単位はm
        :param l1: 　モデル上のJ2からJ3までのリンク長。単位はm
        :param l2: 　モデル上のJ3からJ5までのリンク長。単位はm
        :param min_angles: 　最小角度列。単位はradian
        :param max_angles: 　最大角度列。単位はradian
        """
        # 親クラスの初期化
        super().__init__(server_ip, model=model, dof=dof)

        if min_angles is None:
            self._min_angles = [-3.14] * dof
        else:
            self._min_angles = min_angles
        if max_angles is None:
            self._max_angles = [3.14] * dof
        else:
            self._max_angles = max_angles
        self._h1 = h1
        self._l1 = l1
        self._l2 = l2

        # 位置姿勢ベクトル初期値
        # 上腕３軸の軸構成は共通
        self._x0_v = np.array([1, 0, 0])
        self._y0_v = np.array([0, 1, 0])
        self._z0_v = np.array([0, 0, 1])
        self._x1_v = np.array([1, 0, 0])
        self._y1_v = np.array([0, 1, 0])
        self._z1_v = np.array([0, 0, 1])
        self._x2_v = self._x1_v
        self._y2_v = self._y1_v
        self._z2_v = self._z1_v
        self._x3_v = self._x2_v
        self._y3_v = self._y2_v
        self._z3_v = self._z2_v
        self._p0_v = np.array([0, 0, 0])
        self._p1_v = np.array([0, 0, self._h1])
        self._p2_v = self._p1_v
        self._p3_v = self._p2_v + self._l1 * self._z2_v
        self._p4_v = self._p3_v + self._l2 * self._z3_v

    @abstractmethod
    def calc_actual_angles(self, joints):
        raise NotImplementedError()

    @abstractmethod
    def calc_model_angles(self, joints):
        raise NotImplementedError()

    @abstractmethod
    def calc_FK(self, joints):
        raise NotImplementedError()

    @abstractmethod
    def calc_IK(self, x, y, z):
        raise NotImplementedError()

    @abstractmethod
    def set_pose(self, x, y, z, goal_time, relative=False, interpolation_method=HR4CAPI.MINJERK):
        raise NotImplementedError()

    @abstractmethod
    def get_pose(self):
        raise NotImplementedError()

    def set_joint_reference(self, references, mask=None, relative=False):
        """
        基底クラスにあるメソッドのオーバーライド。位置制御の場合のみモデル角度との間の変換を行う
        :param references: HR4CAPIクラスと同じ
        :param mask: 　　　HR4CAPIクラスと同じ
        :param relative: 　HR4CAPIクラスと同じ
        :return: 　　　　　HR4CAPIクラスと同じ
        """
        target_reference = self._get_reference_with_multiple_control_modes(references, relative=relative)
        if mask is not None:
            mask.extend([1] * (6 - len(mask)))

        super().set_joint_reference(target_reference, mask, relative)

    def set_joint_trajectory(self, goal_references, goal_time,
                             mask=None, relative=False, interpolation_method=HR4CAPI.MINJERK, wait_interpolation=True):
        """
        基底クラスにあるメソッドのオーバーライド。位置制御の場合のみモデル角度との間の変換を行う
        :param goal_references: HR4CAPIクラスと同じ
        :param goal_time: 　　　HR4CAPIクラスと同じ
        :param mask: 　　　　　　HR4CAPIクラスと同じ
        :param relative: 　　　　HR4CAPIクラスと同じ
        :param interpolation_method: 　HR4CAPIクラスと同じ
        :param wait_interpolation: 　　HR4CAPIクラスと同じ
        :return: 　　　　　　　　　　　　HR4CAPIクラスと同じ
        """
        target_reference = self._get_reference_with_multiple_control_modes(goal_references, relative=relative)
        if mask is not None:
            mask.extend([1] * (6 - len(mask)))
        super().set_joint_trajectory(target_reference,
                                     goal_time,
                                     mask,
                                     relative,
                                     interpolation_method,
                                     wait_interpolation)

    def set_control_mode(self, control_modes):
        """
        基底クラスにあるメソッドのオーバーライド。6軸に足りない分を足して基底クラスの同名メソッドに渡す
        :param control_modes: HR4CAPIクラスと同じ
        :return: なし
        """
        control_modes.extend([0] * (6 - self._dof))
        super().set_control_mode(control_modes)

    def get_joint_angle(self):
        """
        基底クラスにあるメソッドのオーバーライド。関節数に応じた分のみ返す
        :return: 関節数分の関節角度列
        """
        joint_angles = super().get_joint_angle()
        return self.calc_model_angles(joint_angles[0:self._dof])

    def get_joint_current(self):
        """
        基底クラスにあるメソッドのオーバーライド。関節数に応じた分のみ返す
        :return: 関節数分の電流値列
        """
        joint_currents = super().get_joint_current()
        return joint_currents[0:self._dof]

    def get_joint_torque(self):
        """
        基底クラスにあるメソッドのオーバーライド。関節数に応じた分のみ返す
        :return: 関節数分の関節トルク列
        """
        joint_torques = super().get_joint_torque()
        return joint_torques[0:self._dof]

    def get_control_mode(self):
        """
        基底クラスにあるメソッドのオーバーライド。関節数に応じた分のみ返す
        :return: 関節数分の制御モード
        """
        control_modes = super().get_control_mode()
        return control_modes[0:self._dof]

    def get_joint_speed(self):
        """
        基底クラスにあるメソッドのオーバーライド。関節数に応じた分のみ返す
        :return: 関節数分の速度列
        """
        joint_speeds = super().get_joint_speed()
        return joint_speeds[0:self._dof]

    def update_uarm_positions(self, j1, j2, j3):
        """
        上腕ユニットのモデル位置を更新する
        :param j1:　J1の関節角度。単位はradian
        :param j2:  J2の関節角度。単位はradian
        :param j3:  J3の関節角度。単位はradian
        :return:　なし
        """
        # update orientation unit vector
        self._x1_v = np.array([math.cos(j1), math.sin(j1), 0])
        self._y1_v = np.array([-math.sin(j1), math.cos(j1), 0])
        self._z1_v = np.array([0, 0, 1.0])

        self._x2_v = math.cos(j2) * self._x1_v - math.sin(j2) * self._z1_v
        self._y2_v = self._y1_v
        self._z2_v = math.sin(j2) * self._x1_v + math.cos(j2) * self._z1_v

        self._x3_v = math.cos(j3) * self._x2_v - math.sin(j3) * self._z2_v
        self._y3_v = self._y2_v
        self._z3_v = math.sin(j3) * self._x2_v + math.cos(j3) * self._z2_v

        # update position of elbow
        self._p3_v = self._p2_v + self._l1 * self._z2_v

        # update position of front end
        self._p4_v = self._p3_v + self._l2 * self._z3_v

    def calc_uarm_FK(self, j1, j2, j3):
        """
        上腕ユニットのFKを解くメソッド
        :param j1:　J1の関節角度。単位はradian
        :param j2:　J2の関節角度。単位はradian
        :param j3:　J3の関節角度。単位はradian
        :return:　デカルト座標系での位置（x, y, z)。単位はm
        """
        self.update_uarm_positions(j1, j2, j3)
        x = self._p4_v[0]
        y = self._p4_v[1]
        z = self._p4_v[2]

        return x, y, z

    def calc_uarm_IK(self, x, y, z):
        """
        上腕ユニットのIKを解くメソッド。デカルト座標系でX軸プラス方向がロボット正面を向く右手座標系で指定。
        :param x:　X座標値。単位はm
        :param y:　Y座標値。単位はm
        :param z:　Z座標値。単位はm
        :return:　順に、IKが成功したか、J1、J2、J3の角度（単位はradian)
        """
        ret = True
        try:
            j1 = math.atan2(y, x)
            a_j3 = (x ** 2 + y ** 2 + (z - self._h1) ** 2 - self._l1 ** 2 - self._l2 ** 2) / (2 * self._l1 * self._l2)
            j3 = math.atan2(math.sqrt(1 - a_j3 ** 2), a_j3)
            a_j2 = math.sqrt(x ** 2 + y ** 2)
            b_j2 = z - self._h1
            m_j2 = self._l1 + self._l2 * math.cos(j3)
            n_j2 = self._l2 * math.sin(j3)
            j2 = math.atan2(m_j2 * a_j2 - n_j2 * b_j2, n_j2 * a_j2 + m_j2 * b_j2)
        except Exception as e:
            LOGGER.error(e)
            LOGGER.error('Failed to solve IK')
            j1 = 0.0
            j2 = 0.0
            j3 = 0.0
            ret = False

        return ret, j1, j2, j3

    def set_uarm_pose(self, x, y, z, goal_time, relative=False, interpolation_method=HR4CAPI.MINJERK):
        """
        上腕ユニット先端部のデカルト座標系での位置を指定して補間移動させる
        :param x:　目標のX座標値。単位はm
        :param y:　目標のY座標値。単位はm
        :param z:　目標のZ座標値。単位はm
        :param goal_time:　目標到達時間。単位はs
        :param relative:　　相対値かどうかのフラグ。デフォルトはFalse
        :param interpolation_method:　補間方法。デフォルトはMINJERK補間。
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
        ret, j1, j2, j3 = self.calc_uarm_IK(tgt_x, tgt_y, tgt_z)
        if ret:
            mask = [0, 0, 0]
            self.set_joint_trajectory([j1, j2, j3], goal_time, mask, False, interpolation_method)

    def get_uarm_pose(self):
        """
        上腕ユニット先端部の現在のデカルト座標系での位置を取得する
        :return:　デカルト座標系での位置（x, y, z)。単位はm
        """
        joint_angles = self.get_joint_angle()
        return self.calc_uarm_FK(*joint_angles)

    def _do_joint_range_validation(self, joint_no, joint_val):
        min_val = self._min_angles[joint_no]
        max_val = self._max_angles[joint_no]
        if joint_val < min_val:
            return min_val
        elif joint_val > max_val:
            return max_val
        else:
            return joint_val

    def _do_range_validation(self, joints):
        mod_joints = []
        for i, jn in enumerate(joints):
            mod_joints.append(self._do_joint_range_validation(i, jn))
        return mod_joints

    def _get_reference_with_multiple_control_modes(self, references, relative=False):
        model_angles = self.get_joint_angle()
        start_actual_angles = self.calc_actual_angles(model_angles)
        control_modes = self.get_control_mode()
        # 位置制御指令のもののみ置き換える
        for i, cm in enumerate(control_modes):
            if cm == HR4CAPI.CONTROLMODE_POSITION:
                if relative:
                    model_angles[i] += references[i]
                else:
                    model_angles[i] = references[i]
        valid_joint_angles = self._do_range_validation(model_angles)
        target_angles = self.calc_actual_angles(valid_joint_angles)
        target_reference = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i, cm in enumerate(control_modes):
            if cm == HR4CAPI.CONTROLMODE_POSITION:
                if relative:
                    target_reference[i] = target_angles[i] - start_actual_angles[i]
                else:
                    target_reference[i] = target_angles[i]
            else:
                target_reference[i] = references[i]

        return target_reference
