#ifndef HR4C_CORE_H
#define HR4C_CORE_H
#include <memory>
#include <yaml-cpp/yaml.h>
#include <motor_controller/motor_control.h>

namespace hr4c
{
    using namespace devices;
    using namespace std;

    // バージョン番号
    constexpr auto LibHR4CCoreVersion = "1.52";

    // 補間方法
    constexpr auto Interpolation_Linear = 0;
    constexpr auto Interpolation_MinJerk = 1;

    // HR4C_coreが外部に提供する全機能に対するアクセスを提供するファサードが実装すべきインタフェース
    class HR4CFacade {
    public:
        // 目標値をセットする
        virtual void setJointReference(vector<double>& ref) = 0;

        // 目標角度列をセットする
        virtual void setJointTrajectory(vector<double>& ref) = 0;

        // 現在値を取得する
        virtual void getJointAngle(vector<double>& res) = 0;

        // 補間中待機する
        virtual void waitInterpolation() = 0;

        // 制御モードを変更する
        virtual void setControlMode(vector<double>& ref) = 0;

        // 1軸サーボオン
        virtual void servoOn(vector<double>& joint_nos) = 0;

        // 全軸サーボオン
        virtual void servoAllOn() = 0;

        // 1軸サーボオフ
        virtual void servoOff(vector<double>& joint_nos) = 0;

        // 全軸サーボオフ
        virtual void servoAllOff() = 0;

        // 現在の制御モードを取得する
        virtual void getControlMode(vector<double>& res) = 0;

        // 現在の電流値を取得する
        virtual void getJointCurrent(vector<double>& res) = 0;

        // 軸ごとにキャリブレーションを行う
        virtual void calibrateJoint(vector<double>& ref) = 0;

        // 軸ごとにキャリブレーションを行うが、現在のエンコーダ値ではなく指定された角度から計算したものを使う
        virtual void calibrateJointFromMemory(vector<double>& ref) = 0;

        // 1軸アラームリセット
        virtual void alarmReset(vector<double>& joint_nos) = 0;

        // 現在のモータ状態を取得する
        virtual void getMotorStatus(vector<double>& res) = 0;

        // 全軸強制停止
        virtual void forceStop(vector<double>& methods) = 0;

        // 現在の速度を取得する
        virtual void getJointSpeed(vector<double>& res) = 0;

        // 現在のトルクを取得する
        virtual void getJointTorque(vector<double>& res) = 0;

        // ログ取得を開始する
        virtual void startLogging() = 0;

        // ログ取得を終了する
        virtual void stopLogging() = 0;

        // ログを全て削除する
        virtual void clearLogs() = 0;

        // ログのファイルパスのリストを返す
        virtual void getLogList(vector<string>& res) = 0;

        // 教示を開始する
        virtual void startTeaching(vector<double>& res) = 0;

        // 教示を終了する
        virtual void stopTeaching(vector<double>& res) = 0;

        // モーションIDを指定して教示動作を再生する
        virtual void replayMotion(vector<double>& motion_id, vector<double>& res) = 0;

        // 記録されているモーションIDのリストを返す
        virtual void getMotionList(vector<double>& res) = 0;

        // 指定したモーションを消去する
        virtual void clearMotion(vector<double>& motion_id, vector<double>& res) = 0;

        // 記録されている全てのモーションを消去する
        virtual void clearAllMotions() = 0;

        // コントローラをシャットダウンする
        virtual void controllerShutdown() = 0;

        // 上位からのpingに対して応答を返す
        virtual void getPong(vector<string>& res) = 0;

        // 全てのセンサ情報を一括で取得する
        virtual void getAllSensorInfo(vector<double>& res) = 0;

        // zerogモードの設定を行う
        virtual void setZeroGMode(vector<double>& ref) = 0;
    };

    //
    // MainControllerが実装すべき抽象インタフェース
    // 実装詳細をクライアントモジュールから隠蔽する
    //
    class MainController : public HR4CFacade
    {
    public:
        // シングルトン
        const static unique_ptr<MainController>& Instance();

        // コントローラを実行する。
        virtual int run(YAML::Node Config, const shared_ptr<MotorControl>& motor_controller) = 0;

        // コントローラをシャットダウンする。
        virtual void shutdown() = 0;

        virtual ~MainController() = default;

    protected:
        MainController() = default;
    };

} // namespace hr4c

#endif // HR4C_CORE_H
