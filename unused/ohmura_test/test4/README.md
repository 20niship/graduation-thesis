# 概要

ロボットに対してC/C++から接続するためのサンプルです。

# 試し方

以下のようにビルドを行い実行ファイルを作成して下さい。
尚、環境変数HR4C_LIBDIRで指定された場所にヘッダファイル（libhr4c_comm.h)
およびシェアードオブジェクトファイル（libhr4c_comm.so）が置かれている必要が
あります。

``` bash
$ mkdir build; cd build
$ cmake ../
$ make
$ ./hr4c_cpp_sample
```

# サンプルの内容

具体的には、sample.cppを見てもらえればと思いますが、
hr4capi_openにてIPアドレス、ポート、タイムアウト時間（ｓ）を指定して
デバイスをオープンします。返り値が正の場合はオープン成功で、返り値を
インスタンス番号として指定することで以後、様々なAPIを呼び出すことが出来ます。
サンプルではhr4capi_start()で通信を開始した後、hr4capi_get_joint_angle()にて
、関節確度を取得し、表示しています。お手元の環境で試す際にはhr4capi_open()で
渡すIPアドレスを適切なものに変更の上、ビルドして下さい。

# ヘッダファイルで定義されている関数群について

Python版のソースコードを見ればだいたい想像がつくと思いますが、簡単なメモを以下に記します。

## int hr4capi_open(char* ip_addr, int controller_port, int timeout, int dof)

  IPアドレス、ポート、通信タイムアウト時間、自由度数を指定してデバイスをオープンする。成功した場合は0以上の値をインスタンス番号として返す。既に同じIPアドレス／ポートの組み合わせが開かれている場合は-1を返す。

## int hr4capi_close(int instance_no)

　インスタンス番号を指定して、デバイスをクローズする。

## int hr4capi_start(int instance_no)

　インスタンス番号を指定して通信を開始する。

## int hr4capi_stop(int instance_no)

　インスタンス番号を指定して通信を終了する。

## void hr4capi_set_joint_reference(int instance_no, double *joint_references, int *masks)

　関節の即時のリファレンス値を設定する。引数はそれぞれインスタンス番号、リファレンス値の格納された配列のポインタ、関節マスクの格納された配列のポインタを渡す。

## void hr4capi_set_joint_trajectory(int instance_no, double *goal_angles, double goal_time, int interpolation_method, int *masks)

　各関節の目標値と到達時間を指定して補間した軌道になるよう制御する。引数はそれぞれインスタンス番号、各関節の目標値の格納された配列のポインタ、目標到達時間（単位は秒）、補間方法、マスク配列のポインタとなる。補間方法は、線形補間0、躍度最小補間1となっている。

## void hr4capi_get_joint_angle(int instance_no, double *joint_angles)

　各関節の角度値（単位ラジアン）を取得する。引数はそれぞれインスタンス番号、返り値の格納される配列のポインタとなる。

## int hr4capi_check_interpolation(int instance_no)

　各関節が補間動作中かどうかを返す。1を返した場合、補間動作中であることを示す。引数はインスタンス番号。

## void hr4capi_set_control_mode(int instance_no, double *control_modes)

　各関節の制御モードを変更する。引数はインスタンス番号および各関節の制御モード。制御モードは、位置制御1、速度制御2、電流制御3、トルク制御4。

## void hr4capi_start_logging(int instance_no)

　インスタンス番号を指定して、ログ取得を開始する。

## void hr4capi_stop_logging(int instance_no)

　インスタンス番号を指定して、ログ取得を終了する。

## void hr4capi_clear_logs(int instance_no)

　インスタンス番号を指定して、ログを全て消去する。

## int hr4capi_get_lognum(int instance_no)

　インスタンス番号を指定して、保存されているログの数を返す。インスタンス番号が不正の場合は-1を返す。

## void hr4capi_get_loglist(int instance_no, char* ret_cstr)

　保存されているログのリストを返す。引数はそれぞれ、インスタンス番号、ログのファイル名が格納された配列のポインタ。

## int hr4capi_get_log(int instance_no, const char* filename)

　ログを取得する。引数はそれぞれインスタンス番号、取得したいログのファイル名。成功した場合は0を返し、失敗した場合は-1を返す。

## void hr4capi_servo_on(int instance_no, int *joint_nos, int size)

　関節のサーボをオンにする。引数はそれぞれ、インスタンス番号、サーボオンにしたい関節番号（0-5)を格納した配列のポインタ、指定する関節数。

## void hr4capi_servo_all_on(int instance_no)

　インスタンス番号を指定して全ての関節のサーボをオンにする。

## void hr4capi_servo_off(int instance_no, int *joint_nos, int size)

　関節のサーボをオフにする。引数はそれぞれ、インスタンス番号、サーボオフにしたい関節番号（0-5)を格納した配列のポインタ、指定する関節数。

## void hr4capi_servo_all_off(int instance_no)

　インスタンス番号を指定して全ての関節のサーボをオフにする。

## void hr4capi_get_control_mode(int instance_no, int* control_mode)

　各関節の制御モードを取得する。引数はインスタンス番号、取得した制御モードが格納される配列のポインタ。

## void hr4capi_get_joint_current(int instance_no, double* joint_current)

　各関節の電流値を取得する（単位はA)。引数はインスタンス番号、取得した電流値が格納される配列のポインタ。

## void hr4capi_calibrate_joint(int instance_no, int joint_no, double calibrate_angle)

　関節を指定してキャリブレーションする。引数はインスタンス番号、指定の関節番号、キャリブレーション角度（ラジアン）。

## void hr4capi_alarm_reset(int instance_no, int *joint_nos, int size)

　関節を指定して、モータのアラームをリセットする。引数はインスタンス番号、リセットする関節番号（0-5）を格納した配列のポインタ、指定する関節数。

## void hr4capi_get_motor_status(int instance_no, int* motor_status)

　モータのステータスを取得する。引数はインスタンス番号、取得したモータステータスを格納する配列のポインタ。

## void hr4capi_force_stop(int instance_no, int *methods, int size)

　関節の強制停止を行う。引数はインスタンス番号、停止方法を格納した配列のポインタ、その配列の要素数。停止方法は0が即時停止、1が減速停止。基本的には要素数は1の配列として渡す。

## void hr4capi_get_joint_speed(int instance_no, double* joint_speed)

　関節の速度を返す。単位はrad/s。引数はインスタンス番号、取得した速度が格納される配列のポインタ。

## void hr4capi_get_joint_torque(int instance_no, double* joint_torque)

　関節のトルクを返す。単位はNm。引数はインスタンス番号、取得したトルクが格納される配列のポインタ。　

## int hr4capi_start_teaching(int instance_no)

　インスタンス番号を指定して、ティーチングを開始する。返り値は、教示開始成功（0)、既に教示中のため失敗（-1)、容量一杯のため失敗（-2)。

## int hr4capi_stop_teaching(int instance_no)

　インスタンス番号を指定して、ティーチングを終了する。返り値は教示したモーションの番号。

## int hr4capi_replay_motion(int instance_no, int replay_id, int* masks)

　指定した教示モーションを再生する。引数はインスタンス番号、モーション番号、関節マスクが格納された配列のポインタ。返り値は、再生成功（0)、指定IDが見つからないため再生失敗（-1)、モーションが登録されていないため再生失敗（-2), モーションIDが不正（-3)。

## void hr4capi_get_motion_list(int instance_no, char* ret_cstr)

　教示され、記憶されているモーション番号のリストを返す。引数はインスタンス番号、モーション番号のリストが格納される配列のポインタ。

## int hr4capi_clear_motion(int instance_no, int clear_id)

　指定されたモーションを消去する。引数はインスタンス番号、消去したいモーション番号。返り値は、消去成功（0)、指定IDが見つからないため消去失敗（-1)、モーションIDが不正（-2)。

## void hr4capi_clear_all_motions(int instance_no)

　インスタンス番号を指定して、全てのモーションを消去する。

## void hr4capi_controller_shutdown(int instance_no)

　インスタンス番号を指定して、該当のコントローラをシャットダウンする。

## int hr4capi_ping(int instance_no)

　インスタンス番号を指定して、対応のコントローラから応答が返るかどうかを確認する。応答が返る場合は1を、そうでない場合は0を返す。

## int hr4capi_update_controller(int instance_no, const char* saved_directory)

　インスタンス番号を指定してコントローラの制御プログラムをアップデートする。引数はインスタンス番号、バイナリの置かれたフォルダパス。返り値は更新成功（0）、更新失敗（-1）。
