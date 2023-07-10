#ifndef HR4C_COMM_H
#define HR4C_COMM_H
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include "command_service.h"
#include "socket_handler.h"


namespace hr4c_comm
{
    using namespace std;
    using namespace socket_api;
    using namespace hr4c_comm::command_service;

    // バージョン番号
    constexpr auto Version = "1.6";

    // 各種定数
    constexpr auto CONTROLLER_USER = "hr4c";
    constexpr auto GREEN_PASS = "GreenM0delK";
    constexpr auto AMBER_PASS = "AmberM0del4";
    constexpr auto ROBOT_MODEL_GREEN = "green";
    constexpr auto ROBOT_MODEL_AMBER = "amber";
    constexpr auto SCP_SEND = 0;
    constexpr auto SCP_RECV = 1;
    constexpr auto BIN_SAVE_DIR = "/home/hr4c/hr4c_core/bin";

    class HR4CSocketClient
    {
     public:
      HR4CSocketClient() = default;
      HR4CSocketClient(string server_ip, int controller_port, int dof, string model) : ip_addr_{std::move(server_ip)},
                                                                                       controller_port_{controller_port},
                                                                                       dof_{dof},
                                                                                       model_{std::move(model)} {
      }
      // 開始する
      int start();

      // 終了する
      int stop();

      // 目標値をセットする
      void setJointReference(vector<double> &ref);

      // 目標軌道を計算・セットする
      void setJointTrajectory(vector<double> &ref);

      // 現在値を取得する
      void getJointAngle(vector<double> &res);

      // 補間中待機する
      void waitInterpolation();

      // 制御モードをセットする
      void setControlMode(vector<double> &ref);

      // ログを開始する
      void startLogging();

      // ログを終了する
      void stopLogging();

      // ログを削除する
      void clearLogs();

      // ログリストを取得する
      void getLogList(vector<string> &res_str);

      // ログファイルを手元にコピーする
      int getLog(string &filename);

      // 1軸サーボオンする
      void servoOn(vector<double> &joint_nos);

      // 全軸サーボオンする
      void servoAllOn();

      // 1軸サーボオフする
      void servoOff(vector<double> &joint_nos);

      // 全軸サーボオフする
      void servoAllOff();

      // 現在の制御モードを取得する
      void getControlMode(vector<double> &res);

      // 現在の電流値を取得する
      void getJointCurrent(vector<double> &res);

      // 1軸ごとのキャリブレーションを手動で行う
      void calibrateJoint(vector<double> &ref);

      // 1軸ごとのキャリブレーションを手動で行う(指定の角度から計算）
      void calibrateJointFromMemory(vector<double> &ref);

      // 1軸アラームリセットする
      void alarmReset(vector<double> &joint_nos);

      // 現在のモータ状態を取得する
      void getMotorStatus(vector<double> &res);

      // 全軸強制停止する
      void forceStop(vector<double> &methods);

      // 現在の速度値を取得する
      void getJointSpeed(vector<double> &res);

      // 現在のトルク値を取得する
      void getJointTorque(vector<double> &res);

      // 教示を開始する
      void startTeaching(vector<double> &res);

      // 教示を終了する
      void stopTeaching(vector<double> &res);

      // MotionIDを指定して教示動作を再生する
      void replayMotion(vector<double> &ref, vector<double> &res);

      // 教示したMotionIDのリストを返す
      void getMotionList(vector<double> &res);

      // MotionIDを指定して教示データを消去する
      void clearMotion(vector<double> &ref, vector<double> &res);

      // 全ての教示動作を消去する
      void clearAllMotions();

      // コントローラをシャットダウンする
      void controllerShutdown();

      // 応答をチェックする
      bool ping();

      // コントローラのバイナリを更新する
      int updateController(string &binary_dir);

      // 全てのセンサ情報を一括で取得する
      void getAllSensorInfo(vector<double> &res);

      // ZeroG Modeの有効、無効を設定する
      void enableZeroGMode(vector<double> &ref);

      // IP, port設定が同じかどうかをチェックする
      bool checkIdentity(const string& server_ip, int port);

      // DOFを返す
      int getDof() { return dof_; };

     private:
      void controllerSetGetCommand(const string& command, vector<double> &ref, vector<double> &res,
                                   vector<string> &res_str, bool check_response);
      void controllerSetCommand(const string& command, vector<double> &ref);
      void controllerGetCommand(const string& command, vector<double> &res, vector<string> &res_str);
      int scpFile(const string& filepath, const int direction);
      int initControllerSocketHandler(string &ip_addr, int port);
      int initCommandService();
      string ip_addr_;
      int controller_port_{};
      int dof_{};
      string model_;
      bool connected_ {false};
      // Commandを通信可能な形態に変換するサービス
      shared_ptr<JSONCommandService> command_service_;
      // Controller Socketのハンドラ
      shared_ptr<SocketHandler> controller_socket_handler_;
    };
} // namespace hr4c_comm

#endif // HR4C_COMM_H
