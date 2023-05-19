# -*- coding: utf-8 -*-
import logging
import numpy as np
import time
import yaml
from hr4c_api.core.hr4c_comm import *

logging.basicConfig()
LOGGER = logging.getLogger(__name__)
LOGGER.setLevel(level=logging.INFO)


class HR4CAPI(object):
    """
    デバイスの一番低レイヤ部分を担う基本クラス
    """
    LINEAR = 0
    MINJERK = 1
    SUDDEN_STOP = 0
    DECELERATION = 1
    CONTROLMODE_POSITION = 1
    CONTROLMODE_SPEED = 2
    CONTROLMODE_CURRENT = 3
    CONTROLMODE_TORQUE = 4
    MOTORSTATUS_SERVO = 'servo'
    MOTORSTATUS_ALARM = 'alarm'
    MOTORSTATUS_LIMIT_ALARM = 'limit_alarm'
    MOTORSTATUS_DEVICE_ERROR = 'device_error'
    MOTORSTATUS_CONNECTION_ERROR = 'connection_error'
    MOTORSTATUS_IN_POSITION = 'in_position'

    def __init__(self, server_ip, model="green", dof=6):
        """
        初期化メソッド
        :param server_ip: コントローラBOX内制御計算機のIPアドレス
        """
        self._dev = None
        self._dof = dof
        config_file_path = hr4c_libdir + "/config.yaml"
        settings = None
        try:
            with open(config_file_path) as f:
                settings = yaml.load(f, Loader=yaml.SafeLoader)
        except OSError as err:
            LOGGER.error('OS error: {0}'.format(err))

        if settings is not None:
            controller_port = int(settings['socket']['controller_port'])
            self._dev = hr4c_comm_so.hr4capi_open(server_ip.encode('ascii'),
                                                  controller_port,
                                                  self._dof,
                                                  model.encode('ascii'))
            if self._dev >= 0:
                connected = hr4c_comm_so.hr4capi_start(self._dev)
            else:
                connected = 0

            # 応答も確認する
            if self._dev >= 0 and connected and self.ping():
                LOGGER.debug('Connection success')
            else:
                LOGGER.error('Connection is already established!')
                if self._dev >= 0:
                    hr4c_comm_so.hr4capi_stop(self._dev)
                    hr4c_comm_so.hr4capi_close(self._dev)
                raise Exception('Connection Error')

    def __del__(self):
        if self._dev >= 0:
            hr4c_comm_so.hr4capi_stop(self._dev)
            hr4c_comm_so.hr4capi_close(self._dev)

    def _get_relative_reference(self, goal_references_np):
        current_control_modes = self.get_control_mode()
        start_angles = self.get_joint_angle()
        start_speeds = self.get_joint_speed()
        start_currents = self.get_joint_current()
        start_torques = self.get_joint_torque()

        start_references = []
        for i, cm in enumerate(current_control_modes):
            if cm == self.CONTROLMODE_POSITION:
                start_references.append(start_angles[i])
            elif cm == self.CONTROLMODE_SPEED:
                start_references.append(start_speeds[i])
            elif cm == self.CONTROLMODE_CURRENT:
                start_references.append(start_currents[i])
            elif cm == self.CONTROLMODE_TORQUE:
                start_references.append(start_torques[i])
            else:
                start_references.append(0.0)
        start_references_np = np.array(start_references)
        goal_references_np += start_references_np

        return goal_references_np

    #
    # 基本動作コマンド
    #
    def set_joint_reference(self, references, mask=None, relative=False, wait_set_reference=True):
        """
        関節制御目標値をセットする。位置／速度／電流制御で共通
        :param references: 関節の制御目標値リスト。位置はradian、速度はrad/s、電流はAの単位
        :param mask: 関節マスクリスト。1の場合はマスクあり、0の場合はマスクなし。Noneの場合は全関節マスクなし。
        :param relative: 相対値指定の場合はTrueを渡す。デフォルトはFalse。
        :param wait_set_reference: リファレンスのセット完了まで待つかどうか。デフォルトはTrue
        :return:　なし
        """
        goal_references_np = np.array(references)
        if relative:
            goal_references_np = self._get_relative_reference(goal_references_np)
        ref_array = goal_references_np.tolist()

        if mask is None:
            mask_list = [0] * self._dof
        else:
            mask_list = mask

        ref_c_array = (ctypes.c_double * len(ref_array))(*ref_array)
        mask_c_array = (ctypes.c_int * len(mask_list))(*mask_list)
        hr4c_comm_so.hr4capi_set_joint_reference(self._dev, ref_c_array, mask_c_array)
        if wait_set_reference:
            self.wait_interpolation()

    def get_joint_angle(self):
        """
        現在の関節角度（radian）を返す
        :return: 関節角度列
        """
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        hr4c_comm_so.hr4capi_get_joint_angle(self._dev, res_c_array)
        results = [rc for rc in res_c_array]

        return results

    def get_joint_current(self):
        """
        現在の電流値（A）を返す
        :return: 関節電流値列
        """
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        hr4c_comm_so.hr4capi_get_joint_current(self._dev, res_c_array)
        results = [rc for rc in res_c_array]

        return results

    def set_joint_trajectory(self, goal_references, goal_time,
                                 mask=None, relative=False, interpolation_method=MINJERK, wait_interpolation=True):
        """
        指定の到達時間（s）で目標値（制御モードにより単位は異なる）まで移動する
        :param goal_references: 関節制御目標値列。位置制御はradian、速度制御はrad/s、電流制御はAの単位となる。
        :param goal_time: 到達時間。単位はs。
        :param mask: 関節マスクリスト。1の場合はマスクあり、0の場合はマスクなし。Noneの場合は全関節マスクなし。
        :param relative: 相対値指定の場合はTrueを渡す。デフォルトはFalse。
        :param interpolation_method: 補間方法を指定する。デフォルトはMINJERK補間。
        :param wait_interpolation: 補間終了まで待機するかどうか。デフォルトはTrue。
        :return: なし
        """
        if interpolation_method is None:
            interpolation_method = self.LINEAR

        goal_references_np = np.array(goal_references)
        if relative:
            goal_references_np = self._get_relative_reference(goal_references_np)
        abs_goal_references = goal_references_np.tolist()

        if mask is None:
            mask_list = [0] * self._dof
        else:
            mask_list = mask

        ref_c_array = (ctypes.c_double * len(abs_goal_references))(*abs_goal_references)
        mask_c_array = (ctypes.c_int * len(mask_list))(*mask_list)
        hr4c_comm_so.hr4capi_set_joint_trajectory(self._dev, ref_c_array, goal_time, interpolation_method, mask_c_array)
        if wait_interpolation:
            self.wait_interpolation()

    def adapt(self, adapt_time=0.5):
        """
        現在のエンコーダ位置を制御目標として動かすことによって馴染む動作を行う。位置制御でない場合は出力を0.0にする。
        :param adapt_time:　馴染み時間（s）。デフォルト値は0.5秒。
        :return: なし
        """
        ja = self.get_joint_angle()
        cmodes = self.get_control_mode()
        for i, cm in enumerate(cmodes):
            if cm != self.CONTROLMODE_POSITION:
                ja[i] = 0.0
        self.set_joint_trajectory(ja, adapt_time)

    def wait_interpolation(self):
        """
        補間動作が終了するのを待機する。
        :return:　なし
        """
        hr4c_comm_so.hr4capi_wait_interpolation(self._dev)

    def servo_on(self, joint_no):
        """
        関節番号を指定してサーボをONにする。
        :param joint_no: 関節番号（0-5)
        :return: なし
        """
        joint_nos = [joint_no]
        jn_c_array = (ctypes.c_int * len(joint_nos))(*joint_nos)
        hr4c_comm_so.hr4capi_servo_on(self._dev, jn_c_array, 1)

    def servo_off(self, joint_no):
        """
        関節番号を指定してサーボをOFFにする。
        :param joint_no: 関節番号（0-5)
        :return: なし
        """
        joint_nos = [joint_no]
        jn_c_array = (ctypes.c_int * len(joint_nos))(*joint_nos)
        hr4c_comm_so.hr4capi_servo_off(self._dev, jn_c_array, 1)

    def servo_all_on(self, fix_mode=False, joint_list=None):
        """
        全ての関節のサーボをONにする。
        :param fix_mode: Trueの場合、 指定した番号の関節を位置制御でサーボONにして固定する。Falseの場合は全軸同時サーボオン
        :param joint_list: 関節番号リスト(default: [0, ..., self._dof - 1])。fix_modeがTrueのときのみ使用
        :return: なし
        """
        # 現在値を最初の制御目標として予めセットしておく
        self.adapt()

        if joint_list is None:
            joint_list = range(self._dof)

        if fix_mode:
            cmodes = self.get_control_mode()
            all_position_mode = [self.CONTROLMODE_POSITION] * self._dof
            if cmodes != all_position_mode:
                self.set_control_mode(all_position_mode)
                time.sleep(0.1)

            for i in joint_list:
                self.servo_on(i)
                time.sleep(0.1)
        else:
            if len(joint_list) == self._dof:
                hr4c_comm_so.hr4capi_servo_all_on(self._dev)
            else:
                for i in joint_list:
                    self.servo_on(i)

    def servo_all_off(self):
        """
        全ての関節のサーボをOFFにする。
        :return: なし
        """
        hr4c_comm_so.hr4capi_servo_all_off(self._dev)

    def set_control_mode(self, control_modes, no_check=False, retry_count=3):
        """
        各関節の制御モード（1: 位置制御、2: 速度制御、3: 電流制御、4: トルク制御）を指定する。
        :param control_modes: 各関節の制御モード
        :param no_check: セット後のチェックを行わない場合はTrue。デフォルトはFalse。
        :param retry_count: 正しくセットされてなかった場合の最大リトライ回数。デフォルト値は3。
        :return: なし
        """
        current_cmodes = self.get_control_mode()
        ref_c_array = (ctypes.c_int * len(control_modes))(*control_modes)
        try_count = retry_count + 1

        for i in range(try_count):
            if current_cmodes != control_modes:
                hr4c_comm_so.hr4capi_set_control_mode(self._dev, ref_c_array)
                if no_check:
                    return
                else:
                    time.sleep(0.3)
                    current_cmodes = self.get_control_mode()
            else:
                # 正しく設定できた場合は抜ける
                return

        if retry_count != 0:
            # 繰り返し回数分試行してもうまくいかない場合
            LOGGER.warning('Failed to set mode, Abort: ' + str(current_cmodes))

    def get_control_mode(self):
        """
        各関節の制御モードを返す。
        :return: 制御モード列
        """
        res_array = [0] * self._dof
        res_c_array = (ctypes.c_int * len(res_array))(*res_array)
        hr4c_comm_so.hr4capi_get_control_mode(self._dev, res_c_array)
        results = [rc for rc in res_c_array]

        return results

    def calibrate_joint(self, joint_no, calibration_angle):
        """
        手動で関節毎のキャリブレーションを行う
        :param joint_no: 関節番号（0-5)
        :param calibration_angle: 現在の姿勢がモデル上のどの角度に対応しているか（radian）
        :return: なし
        """
        hr4c_comm_so.hr4capi_calibrate_joint(self._dev, joint_no, calibration_angle)

    def calibrate_joint_from_memory(self, joint_no, calibration_angle, memory_angle):
        """
        手動で関節毎のキャリブレーションを行う(キャリブレーションを行う際の角度を指定）
        :param joint_no: 関節番号（0-5)
        :param calibration_angle: 現在の姿勢がモデル上のどの角度に対応しているか（radian）
        :param memory_angle: キャリブレーションを行わせる位置での現在の設定での角度（radian）
        :return: なし
        """
        hr4c_comm_so.hr4capi_calibrate_joint_from_memory(self._dev, joint_no, calibration_angle, memory_angle)

    def get_joint_speed(self):
        """
        現在の関節速度（rad/s)を返す
        :return: 関節速度列
        """
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        hr4c_comm_so.hr4capi_get_joint_speed(self._dev, res_c_array)
        results = [rc for rc in res_c_array]

        return results

    def get_joint_torque(self):
        """
        現在の関節トルク（Nm）を返す
        :return: 関列トルク列
        """
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        hr4c_comm_so.hr4capi_get_joint_torque(self._dev, res_c_array)
        results = [rc for rc in res_c_array]

        return results

    def alarm_reset(self, joint_no):
        """
        関節毎のアラームリセットを行う
        :param joint_no: 関節番号 (0-5)
        :return: なし
        """
        joint_nos = [joint_no]
        jn_c_array = (ctypes.c_int * len(joint_nos))(*joint_nos)
        hr4c_comm_so.hr4capi_alarm_reset(self._dev, jn_c_array, 1)

    def get_motor_status(self):
        """
        各モータの状態を辞書型で返す
        :return: サーボ完了状態、サーボ状態、アラーム状態、リミットアラーム状態、デバイスエラー、接続エラーを0/1で表現した辞書オブジェクト
        """
        res_array = [0] * self._dof
        res_c_array = (ctypes.c_int * len(res_array))(*res_array)
        hr4c_comm_so.hr4capi_get_motor_status(self._dev, res_c_array)
        results = [rc for rc in res_c_array]

        motor_status_dict = {}
        motor_status_dict[self.MOTORSTATUS_IN_POSITION] = [(rc & 0x20) >> 5 for rc in results]
        motor_status_dict[self.MOTORSTATUS_SERVO] = [(rc & 0x10) >> 4 for rc in results]
        motor_status_dict[self.MOTORSTATUS_ALARM] = [(rc & 0x08) >> 3 for rc in results]
        motor_status_dict[self.MOTORSTATUS_LIMIT_ALARM] = [(rc & 0x04) >> 2 for rc in results]
        motor_status_dict[self.MOTORSTATUS_DEVICE_ERROR] = [(rc & 0x02) >> 1 for rc in results]
        motor_status_dict[self.MOTORSTATUS_CONNECTION_ERROR] = [(rc & 0x01) for rc in results]

        return motor_status_dict

    def get_diagnosis(self):
        """
        モジュールの系としての状態を診断し、文字列で返す。
        'WARN' -> software復帰可能な状態、'ERROR' -> softwareでの復帰が出来ない状態
        :return: 'OK', 'WARN', 'ERROR'のいずれか
        """
        motor_status_dict = self.get_motor_status()
        if any(motor_status_dict[self.MOTORSTATUS_CONNECTION_ERROR]) or any(motor_status_dict[self.MOTORSTATUS_LIMIT_ALARM]):
            return 'ERROR'
        elif any(motor_status_dict[self.MOTORSTATUS_ALARM]) or any(motor_status_dict[self.MOTORSTATUS_DEVICE_ERROR]):
            return 'WARN'
        else:
            return 'OK'

    def force_stop(self, method=DECELERATION):
        """
        set_trajectoryにてwait_interpolation=Falseの場合、補間動作中に割り込んで強制停止させる。
        :param method: 停止方法を指定する。デフォルトは減速停止。
        :return: なし
        """
        methods = [method]
        me_c_array = (ctypes.c_int * len(methods))(*methods)
        hr4c_comm_so.hr4capi_force_stop(self._dev, me_c_array, 1)

    def start_logging(self):
        """
        ログの記録を開始する
        :return: なし
        """
        hr4c_comm_so.hr4capi_start_logging(self._dev)

    def stop_logging(self):
        """
        ログの記録を終了する
        :return: なし
        """
        hr4c_comm_so.hr4capi_stop_logging(self._dev)

    def clear_logs(self):
        """
        記録されているログファイルを全て消去する
        :return: なし
        """
        hr4c_comm_so.hr4capi_clear_logs(self._dev)

    def get_log_number(self):
        """
        記録されているログファイルの数を返す
        :return: ログファイルの数
        """
        return hr4c_comm_so.hr4capi_get_lognum(self._dev)

    def get_log_list(self):
        """
        記録されているログファイルのファイルパスを列挙する
        :return: ファイルパスのリスト
        """
        logfn_no = self.get_log_number()
        if logfn_no == 0:
            return []

        loglist_c_str = ctypes.create_string_buffer(255)
        hr4c_comm_so.hr4capi_get_loglist(self._dev, loglist_c_str)
        loglist_str = loglist_c_str.value.decode('utf-8')
        log_lists = [x for x in loglist_str.split(',') if x != '']

        return list(set(log_lists))

    def get_log(self, logfile_name):
        """
        ログファイル名を指定し、プログラム実行場所にコピーする
        :param logfile_name: ログファイル名
        :return: 0(成功）、-1（失敗）
        """
        result = hr4c_comm_so.hr4capi_get_log(self._dev, logfile_name.encode('utf-8'))
        
        return result

    def start_force_control(self, id_list, mode=None, fref=0):
        """
        複数の関節をトルク制御もしくは電流制御にする
        :param id_list: 力制御にする関節のリスト
        :param mode: トルク制御か電流制御か指定（default: 4 トルク制御）
        :return: なし
        """
        if mode is None:
            mode_no = self.CONTROLMODE_TORQUE
        else:
            mode_no = mode
        cmodes = self.get_control_mode()
        ref = [fref] * len(cmodes)
        mask = [1]*len(cmodes)
        for i in id_list:
            self.servo_off(i)
            cmodes[i] = mode_no
            mask[i] = 0
        self.set_control_mode(cmodes)
        self.set_joint_reference(ref, mask=mask)
        for i in id_list:
            self.servo_on(i)

    def controller_shutdown(self):
        """
        コントローラのシャットダウンを行う。シャットダウンコマンド後は、コントローラの電源が落ちるので、再起動するには再度
        本体電源を入れ直す必要がある。
        :return: なし
        """
        hr4c_comm_so.hr4capi_controller_shutdown(self._dev)

    #
    # 基本教示コマンド
    #
    def start_teaching(self):
        """
        教示を開始する
        :return: 教示開始成功（0)、既に教示中のため失敗（-1)、容量一杯のため失敗（-2)
        """
        result = hr4c_comm_so.hr4capi_start_teaching(self._dev)
        return result

    def stop_teaching(self):
        """
        教示を終了する
        :return: 教示モーション番号
        """
        result = hr4c_comm_so.hr4capi_stop_teaching(self._dev)
        return result

    def replay_motion(self, motion_id, mask=None, wait_replay_finished=True):
        """
        教示動作を再生する
        :param motion_id: 教示モーション番号
        :param mask: 関節マスクリスト。1の場合はマスクあり、0の場合はマスクなし。Noneの場合は全関節マスクなし。
        :param wait_replay_finished: 再生終了まで待機するかどうか。デフォルトはTrue。
        :return: 再生成功（0)、指定IDが見つからないため再生失敗（-1)、モーションが登録されていないため再生失敗（-2), モーションIDが不正（-3)
        """
        if type(motion_id) is not int:
            LOGGER.error('Invalid motion_id: {0}'.format(motion_id))
            return -3
        if mask is None:
            mask_list = [0] * self._dof
        else:
            mask_list = mask
        mask_c_array = (ctypes.c_int * len(mask_list))(*mask_list)
        result = hr4c_comm_so.hr4capi_replay_motion(self._dev, motion_id, mask_c_array)

        if result == 0 and wait_replay_finished:
            self.wait_interpolation()

        return result

    def get_motion_list(self):
        """
        記録済みのmotion_idリストを返す
        :return: 教示モーション番号のリスト
        """
        motionid_c_str = ctypes.create_string_buffer(32)
        hr4c_comm_so.hr4capi_get_motion_list(self._dev, motionid_c_str)
        motionid_str = motionid_c_str.value.decode('utf-8')
        motion_id_list = [int(x) for x in motionid_str.split(',') if x != '']

        return motion_id_list

    def clear_motion(self, motion_id):
        """
        指定の教示動作を消去する
        :param motion_id: 教示モーション番号
        :return: 消去成功（0)、指定IDが見つからないため消去失敗（-1)、モーションIDが不正（-2)
        """
        if type(motion_id) is not int:
            LOGGER.error('Invalid motion_id: {0}'.format(motion_id))
            return -2
        result = hr4c_comm_so.hr4capi_clear_motion(self._dev, motion_id)
        return result

    def clear_all_motions(self):
        """
        教示動作を全て消去する
        :return: なし
        """
        hr4c_comm_so.hr4capi_clear_all_motions(self._dev)

    def ping(self):
        """
        正しい応答が返ってきているかどうかを確認するメソッド
        :return: 0: 応答なし、1: 応答あり
        """
        return hr4c_comm_so.hr4capi_ping(self._dev)

    def update_controller_software(self, bin_dir):
        """
        コントローラの実行ファイルおよび設定ファイルを更新する
        :param bin_dir: 実行ファイル及び設定ファイルの配置されたディレクトリのパス
        :return: 更新成功（0）、更新失敗（-1）
        """
        return hr4c_comm_so.hr4capi_update_controller(self._dev, bin_dir.encode('utf-8'))

    def enable_zerog_mode(self, on_off):
        """
        zrog モードの有効、無効を指示する。モードを有効にする際にはJ2/J3をトルクモードにする
        :param on_off:　有効（True), 無効(False) を指示
        :return: なし
        """
        if on_off:
            cmode = self.get_control_mode()
            cmode[0] = self.CONTROLMODE_TORQUE
            cmode[1] = self.CONTROLMODE_TORQUE
            cmode[2] = self.CONTROLMODE_TORQUE
            self.set_control_mode(cmode)
            hr4c_comm_so.hr4capi_enable_zerog_mode(self._dev, 1)
        else:
            hr4c_comm_so.hr4capi_enable_zerog_mode(self._dev, 0)

