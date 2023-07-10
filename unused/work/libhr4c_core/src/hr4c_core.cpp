#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include "algorithm/linear_interpolator.h"
#include "algorithm/minjerk_interpolator.h"
#include "motor_controller/motor_state.h"
#include "net/socketservice_handler.h"
#include "services/ddm_control_service.h"
#include "utils/hr4c_utils.h"
#include "hr4c_core.h"


namespace hr4c
{
    using namespace algorithm;
    using namespace services;
    using namespace socket_api;
    using namespace utils;

    constexpr auto LOG_EXTENTION = ".log";
    constexpr auto BIN_EXTENTION = ".bin";
    constexpr auto TEACHING_OK = 0.0;
    constexpr auto TEACHING_FAILED_ALREADY_TEACHING = -1.0;
    constexpr auto TEACHING_FAILED_MOTION_FULL = -2.0;
    constexpr auto TEACHING_FAILED_NO_SUCH_ID = -1.0;
    constexpr auto TEACHING_FAILED_NO_MOTION = -2.0;
    constexpr auto TEACHING_FAILED_INVALID_MOTION = -3.0;
    constexpr auto DEFAULT_START_REPLAY_MOTION_TIME = 2.0;
    constexpr auto TEACHING_MOTION_DIR = "motions";
    constexpr auto TEACHING_MOTION_BIN_BASENAME = "teaching_motions_";
    constexpr auto TEACHING_MOTION_BIN_EXT = ".bin";

    class DefaultMainController final : public MainController {
    public:
        int run(const YAML::Node config, const shared_ptr<MotorControl>& motor_controller) override;
        void shutdown() override;

        // HR4CFacadeの実装
        // 目標値をセットする
        void setJointReference(vector<double>& ref) override;
        // 目標角度列をセットする
        void setJointTrajectory(vector<double>& ref) override;
        // 現在値を取得する
        void getJointAngle(vector<double>& res) override;
        // 補間中待機する
        void waitInterpolation() override;
        // 制御モードを変更する
        void setControlMode(vector<double>& ref) override;
        // 1軸サーボオン
        void servoOn(vector<double>& joint_nos) override;
        // 全軸サーボオン
        void servoAllOn() override;
        // 1軸サーボオフ
        void servoOff(vector<double>& joint_nos) override;
        // 全軸サーボオフ
        void servoAllOff() override;
        // 現在の制御モードを取得する
        void getControlMode(vector<double>& res) override;
        // 現在の電流値を取得する
        void getJointCurrent(vector<double>& res) override;
        // 軸ごとにキャリブレーションを行う
        void calibrateJoint(vector<double>& ref) override;
        // 軸ごとにキャリブレーションを行うが、現在のエンコーダ値ではなく指定された角度から計算したものを使う
        void calibrateJointFromMemory(vector<double>& ref) override;
        // 1軸アラームリセット
        void alarmReset(vector<double>& joint_nos) override;
        // 現在のモータ状態を取得する
        void getMotorStatus(vector<double>& res) override;
        // 全軸強制停止
        void forceStop(vector<double>& methods) override;
        // 現在の速度を取得する
        void getJointSpeed(vector<double>& res) override;
        // 現在のトルクを取得する
        void getJointTorque(vector<double>& res) override;
        // ログ取得を開始する
        void startLogging() override;
        // ログ取得を終了する
        void stopLogging() override;
        // ログを全て削除する
        void clearLogs()  override;
        // ログのファイルパスのリストを返す
        void getLogList(vector<string>& res) override;
        // 教示を開始する
        void startTeaching(vector<double>& res) override;
        // 教示を終了する
        void stopTeaching(vector<double>& res) override;
        // モーションIDを指定して教示動作を再生する
        void replayMotion(vector<double>& motion_id, vector<double>& res) override;
        // 記録されているモーションIDのリストを返す
        void getMotionList(vector<double>& res) override;
        // 指定したモーションを消去する
        void clearMotion(vector<double>& motion_id, vector<double>& res) override;
        // 記録されている全てのモーションを消去する
        void clearAllMotions() override;
        // コントローラをシャットダウンする
        void controllerShutdown() override;
        // 上位からのpingに対して応答を返す
        void getPong(vector<string>& res) override;
        // 全てのセンサ情報を一括で取得する
        void getAllSensorInfo(vector<double>& res) override;
        // zerogモードの設定を行う
        void setZeroGMode(vector<double>& ref) override;

    private:
        // 制御周期
        int ControlFrequency{};
        YAML::Node Config;

        // for singleton pattern
        friend MainController;
        friend unique_ptr<MainController>;
        DefaultMainController() = default;
        int initSocketHandler();
        int initServices(const shared_ptr<MotorControl>& motor_controller);
        int initSystemLog();
        int initTeachingFunctions();
        bool calcJointTrajectory(vector<double>& goal_references,
                                 double goal_time,
                                 int interpolation_method,
                                 const shared_ptr<vector<int>>& masks,
                                 const shared_ptr<deque<vector<double>>>& ref_array);
        int getVacantMotionId();
        void synchronizedControlLoop();
        string makeMotorStateLogString(chrono::system_clock::time_point start);
        void recordUserLog(chrono::system_clock::time_point start);
        void recordTeachingMotion(vector<MotorState>& mstate_vector);
        void writeUserLogToFile();
        void finishTeaching();
        void writeMotionDataToBin(int teaching_id);
        void readMotionDataFromBin(int teaching_id);
        atomic<bool> loop_flag_ {true};
        shared_ptr<vector<MotorState> > current_motor_states_;
        vector<double> last_targets_;
        vector<spdlog::sink_ptr> logger_sinks_;
        shared_ptr<spdlog::logger> combined_logger_;

        // 各ジョブを実際に実行するサービス
        unique_ptr<DDMControlService> ddm_control_service_;
        // SocketServerのハンドラ
        unique_ptr<SocketServiceHandler> socketservice_handler_;

        bool userlog_record_flag_{};
        long userlog_max_count_{};
        long userlog_count_{};
        string userlog_save_dir_;
        vector<string> userlogs_;

        vector<vector<vector<double>>> teaching_motions_;
        long max_teaching_data_num_{};
        int max_teaching_motion_num_{};
        int teaching_id_{};
        bool teaching_flag_{};
        shared_ptr<MotorControl> motor_controller_{};
    };

} // namespace hr4c

using namespace hr4c;

const unique_ptr<MainController>& MainController::Instance() {
    static const auto instance = unique_ptr<MainController> {new DefaultMainController};
    return instance;
};


int DefaultMainController::run(const YAML::Node config, const shared_ptr<MotorControl>& motor_controller)
{
    // コンフィグ情報の格納
    Config = config;

    // 制御周期の読み込み
    ControlFrequency = Config["system"]["control_frequency"].as<int>();
    spdlog::info("ControlFrequency: " + to_string(ControlFrequency) + "Hz");

    // 初期値セット
    motor_controller_ = motor_controller;
    auto joint_names = Config["joints"]["joint_names"].as<vector<string>>();
    current_motor_states_ = make_shared<vector<MotorState> >();
    for (auto i = 0; i < joint_names.size(); ++i) {
        current_motor_states_->emplace_back(MotorState());
        last_targets_.emplace_back(0.0);
    }
    motor_controller_->setMotorParams(Config["joints"]);

    // デフォルトログの設定
    initSystemLog();

    // 教示用設定
    initTeachingFunctions();

    // Socketサーバハンドラを初期化
    initSocketHandler();

    // 各サービスを初期化
    initServices(motor_controller);

    // 同期制御を開始
    thread cth(&DefaultMainController::synchronizedControlLoop, this);

    // スレッド終了待機
    cth.join();

    return 0;
}

void DefaultMainController::shutdown()
{
    loop_flag_ = false;

    // socket server shutdown
    socketservice_handler_->shutdownServer();

    // 終了時は必ずサーボオフする
    servoAllOff();

    // shutdown services
    ddm_control_service_->shutdownDDMControllers();
}

void DefaultMainController::setJointReference(vector<double>& ref) {
    spdlog::debug("SetJointReference");
    auto joint_num = motor_controller_->getJointNum();
    shared_ptr<deque<vector<double> > > ref_array;
    ref_array = make_shared<deque<vector<double> > >();
    vector<double> targets{};
    copy(ref.begin(), ref.begin() + joint_num, back_inserter(targets));
    ref_array->emplace_back(targets);
    shared_ptr<vector<int> > masks;
    masks = make_shared<vector<int> >();
    copy(ref.begin() + joint_num, ref.begin() + joint_num + joint_num, back_inserter(*masks));
    ddm_control_service_->setJointTargetsWhenReady(ref_array, masks);
}

bool DefaultMainController::calcJointTrajectory(vector<double>& goal_references,
                                                double goal_time,
                                                int interpolation_method,
                                                const shared_ptr<vector<int>>& masks,
                                                const shared_ptr<deque<vector<double>>>& ref_array) {
    auto joint_num = motor_controller_->getJointNum();
    vector<double> start_references{};
    start_references.reserve(joint_num);

    shared_ptr<Interpolator> interpolator;

    for (auto i = 0; i < joint_num; i++) {
        switch (current_motor_states_->at(i).control_mode) {
            case ControlModePosition:
                start_references[i] = current_motor_states_->at(i).angle;
                break;
            case ControlModeSpeed:
                start_references[i] = current_motor_states_->at(i).speed;
                break;
            case ControlModeCurrent:
                start_references[i] = current_motor_states_->at(i).current;
                break;
            case ControlModeTorque:
                start_references[i] = current_motor_states_->at(i).torque;
                break;
            default:
                start_references[i] = current_motor_states_->at(i).angle;
                break;
        }
    }

    double tick_s = 1.0 / ControlFrequency;

    switch (interpolation_method) {
        case Interpolation_Linear:
            interpolator = make_shared<LinearInterpolator>();
            break;
        case Interpolation_MinJerk:
            interpolator = make_shared<MinJerkInterpolator>();
            break;
        default:
            // おかしな設定の場合は抜ける
            cerr << "Unsupported interpolation method" << endl;
            return false;
    }

    vector<vector<double>> interpolated_results{};
    interpolated_results.resize(joint_num);
    for (auto i = 0; i < joint_num; ++i) {
        interpolator->interpolate(start_references[i], goal_references[i], goal_time, tick_s, interpolated_results[i]);
    }

    SetJointArrayToTrajectory(interpolated_results, ref_array);

    return true;
}

void DefaultMainController::setJointTrajectory(vector<double>& ref) {
    spdlog::info("SetJointTrajectory");
    auto joint_num = motor_controller_->getJointNum();
    vector<double> goal_references{};
    copy(ref.begin(), ref.begin() + joint_num, back_inserter(goal_references));
    double goal_time = ref[joint_num];
    if (goal_time <= 0.0) {
        spdlog::warn("Goal time is not valid: " + to_string(goal_time));
        return;
    }

    int interpolation_method = static_cast<int>(ref[joint_num + 1]);
    shared_ptr<vector<int> > masks;
    masks = make_shared<vector<int> >();
    copy(ref.begin() + joint_num + 2, ref.begin() + joint_num + 2 + joint_num, back_inserter(*masks));
    shared_ptr<deque<vector<double> > > ref_array = make_shared<deque<vector<double> > >();
    auto ret = calcJointTrajectory(goal_references, goal_time, interpolation_method, masks, ref_array);
    if (ret) {
        ddm_control_service_->setJointTargetsWhenReady(ref_array, masks);
    }
}

void DefaultMainController::getJointAngle(vector<double>& res) {
    spdlog::debug("GetJointAngle");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).angle);
    }
}

void DefaultMainController::waitInterpolation() {
    spdlog::info("WaitInterpolation");
    ddm_control_service_->waitInterpolation();
}

void DefaultMainController::setControlMode(vector<double>& ref) {
    spdlog::info("setControlMode");
    auto errcnt = 5;
    auto cnt = 0;
    auto joint_num = motor_controller_->getJointNum();
    vector<int> cmodes{};
    for (auto i = 0; i < joint_num; ++i) {
        cmodes.emplace_back(static_cast<int>(ref[i]));
    }

    ddm_control_service_->setControlModeWhenReady(cmodes, *current_motor_states_, last_targets_);
}

void DefaultMainController::servoOn(vector<double>& joint_nos) {
    for (auto jn : joint_nos) {
        int joint_no = int(jn);
        if (joint_no < 0 || joint_no >= last_targets_.size()) {
            spdlog::error("Invalid joint_no: " + to_string(joint_no));
            return;
        }
        ddm_control_service_->servoOn(joint_no, last_targets_, current_motor_states_->at(joint_no));
    }
}

void DefaultMainController::servoAllOn() {
    ddm_control_service_->servoAllOn(last_targets_, *current_motor_states_);
}

void DefaultMainController::servoOff(vector<double>& joint_nos) {
    for (auto jn : joint_nos) {
        int joint_no = int(jn);
	if (joint_no < 0 || joint_no >= last_targets_.size()) {
	    spdlog::error("Invalid joint_no: " + to_string(joint_no));
	    return;
	}
        ddm_control_service_->servoOff(joint_no, current_motor_states_->at(joint_no));
    }
}

void DefaultMainController::servoAllOff() {
    ddm_control_service_->servoAllOff(*current_motor_states_);
}

int DefaultMainController::initSocketHandler() {
    auto port = Config["socket"]["port"].as<int>();
    socketservice_handler_ = make_unique<SocketServiceHandler>(port, *this);

    // Socketサーバの初期化
    socketservice_handler_->initServer();

    // Socketサーバを別スレッドで実行する
    socketservice_handler_->run();

    return 0;
}

int DefaultMainController::initServices(const shared_ptr<MotorControl>& motor_controller) {
    // DDMControlService
    ddm_control_service_ = make_unique<DDMControlService>(motor_controller);

    return 0;
}

int DefaultMainController::initSystemLog()
{
    // debug用のlog
    auto logconfig = Config["log"];
    auto db_max_size_mb = logconfig["debug_log"]["max_size_mb"].as<int>();
    auto rotate_num = logconfig["debug_log"]["rotate_num"].as<int>();

    logger_sinks_.push_back(make_shared<spdlog::sinks::rotating_file_sink_mt>
                                    ("logs/debug.log",
                                     db_max_size_mb * 1024 * 1024,
                                     rotate_num, false));
    logger_sinks_.push_back(make_shared<spdlog::sinks::stdout_sink_mt>());
    combined_logger_ = std::make_shared<spdlog::logger>("systemlog", begin(logger_sinks_),
                                                            end(logger_sinks_));

    // log levelの設定
    auto log_level_str = logconfig["debug_log"]["log_level"].as<string>();
    spdlog::level::level_enum log_level;
    if(log_level_str == "off") {
        log_level = spdlog::level::off;
    } else if(log_level_str == "trace") {
        log_level = spdlog::level::trace;
    } else if(log_level_str == "debug") {
        log_level = spdlog::level::debug;
    } else if(log_level_str == "info") {
        log_level = spdlog::level::info;
    } else if(log_level_str == "warn") {
        log_level = spdlog::level::warn;
    } else if(log_level_str == "err") {
        log_level = spdlog::level::err;
    } else if(log_level_str == "critical") {
        log_level = spdlog::level::critical;
    } else {
        log_level = spdlog::level::info;
    }
    combined_logger_->set_level(log_level);

    spdlog::register_logger(combined_logger_);
    spdlog::set_default_logger(combined_logger_);

    // user用のlog設定
    userlog_max_count_ = static_cast<long>(logconfig["user_log"]["max_sec"].as<int>()) * static_cast<long>(ControlFrequency);
    userlog_save_dir_ = logconfig["user_log"]["save_dir"].as<string>();
    userlog_record_flag_ = false;
    userlog_count_ = 0;

    return 0;
}

int DefaultMainController::initTeachingFunctions() {

    auto teaching_config = Config["teaching"];
    max_teaching_motion_num_ = teaching_config["max_motion_num"].as<int>();
    max_teaching_data_num_ = static_cast<long>(teaching_config["max_sec"].as<int>()) * static_cast<long>(ControlFrequency);

    teaching_motions_.resize(max_teaching_motion_num_);

    for (auto tm: teaching_motions_) {
        tm.reserve(max_teaching_data_num_);
    }

    // ファイル保存場所がなければ作成する
    std::filesystem::create_directory(TEACHING_MOTION_DIR);

    // 既存モーションファイルがあれば読み込み
    for (const auto& file : std::filesystem::directory_iterator(TEACHING_MOTION_DIR)) {
        if( file.path().extension() == BIN_EXTENTION) {
            string basename_str = string(TEACHING_MOTION_BIN_BASENAME);
            auto motion_id = stoi(file.path().stem().string().substr(basename_str.size()));
            spdlog::info("Read teaching motion: motion_id(" + to_string(motion_id) + ")");
            readMotionDataFromBin(motion_id);
        }
    }

    teaching_id_ = -1;
    teaching_flag_ = false;

    return 0;
}

string DefaultMainController::makeMotorStateLogString(chrono::system_clock::time_point start)
{
    stringstream ss;
    string targets, angles, currents, alarms, lalarms, servos, control_modes, speeds, torques, seq_no;
    for (auto tgt : last_targets_) {
        targets += to_string(tgt) + ":";
    }

    for (auto& mstate : *current_motor_states_) {
        angles += to_string(mstate.angle) + ":";
        currents += to_string(mstate.current) + ":";
        alarms += to_string(mstate.alarm) + ":";
        lalarms += to_string(mstate.limit_alarm) + ":";
        servos += to_string(mstate.servo_state) + ":";
        control_modes += to_string(mstate.control_mode) + ":";
        speeds += to_string(mstate.speed) + ":";
        torques += to_string(mstate.torque) + ":";
        // seq_noは共通
        seq_no = to_string(mstate.seq_no);
    }

    // 現在時刻、最新の指示値、関節角度、電流値、アラーム、リミットアラーム、サーボ, 制御モード、速度、トルク、シーケンス番号の順。
    // 各項目間の区切りはカンマ、項目内での区切りはコロン
    time_t start_time = chrono::system_clock::to_time_t(start);
    struct tm tm{};
    localtime_r(&start_time, &tm);
    chrono::microseconds microsec = chrono::duration_cast<chrono::microseconds>(start.time_since_epoch());
    size_t fractional_seconds = microsec.count() % 1000000;

    ss << tm.tm_hour << ":" << tm.tm_min << ":" << tm.tm_sec << "." ;
    ss << fractional_seconds << ",";
    ss << targets + ",";
    ss << angles + ",";
    ss << currents + ",";
    ss << alarms + ",";
    ss << lalarms + ",";
    ss << servos + ",";
    ss << control_modes + ",";
    ss << speeds + ",";
    ss << torques + ",";
    ss << seq_no;

    return ss.str();
}

void DefaultMainController::recordUserLog(chrono::system_clock::time_point start) {
    if (userlog_count_ < userlog_max_count_) {
        // log書き出し用データ作成
        string log_str = makeMotorStateLogString(start);
        userlogs_.push_back(log_str);
        userlog_count_ += 1;
    } else {
        spdlog::error("Exceeded maximum log size!");
    }
}

void DefaultMainController::recordTeachingMotion(vector<MotorState>& mstate_vector) {
    if (teaching_id_ >= 0 && teaching_id_ < max_teaching_motion_num_) {
        auto joint_num = motor_controller_->getJointNum();
        vector<double> recorded_angle{};
        recorded_angle.reserve(joint_num);
        for (auto i = 0; i < joint_num; ++i) {
            recorded_angle.emplace_back(mstate_vector.at(i).angle);
        }

        if (teaching_motions_.at(teaching_id_).size() < max_teaching_data_num_) {
            teaching_motions_.at(teaching_id_).push_back(recorded_angle);
        } else {
            spdlog::error("Size is beyond the maximum limit for one teaching motion!, stops teaching");
            finishTeaching();
        }
    }
}

void DefaultMainController::writeUserLogToFile() {
    time_t start_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    struct tm tm{};
    localtime_r(&start_time, &tm);
    auto date_str = to_string(tm.tm_year + 1900) + to_string(tm.tm_mon + 1) +  to_string(tm.tm_mday);
    auto time_str = to_string(tm.tm_hour) + ":" + to_string(tm.tm_min) + ":" + to_string(tm.tm_sec);
    auto save_file_path = userlog_save_dir_ + date_str + "_" + time_str + LOG_EXTENTION;

    std::ofstream output_file(save_file_path);
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(userlogs_.begin(), userlogs_.end(), output_iterator);

    // ログファイル名が被らないように一度書き出したら最低1秒間は間をあける
    sleep(1.0);

    // カウンタ等をクリアする
    userlog_count_ = 0;
    userlogs_.clear();
}

void DefaultMainController::synchronizedControlLoop()
{
    // 周期ループ
    auto control_tick_ns = 1000 * 1000 * 1000 / ControlFrequency;
    chrono::system_clock::time_point start;

    start = chrono::system_clock::now();
    while (loop_flag_) {
        // motorにアクセスして制御を行う
        ddm_control_service_->controlDDM(last_targets_, current_motor_states_);

        // log出力
        if (userlog_record_flag_) {
            recordUserLog(start);
        }

        // 教示
        if (teaching_flag_) {
            recordTeachingMotion(*current_motor_states_);
        }

        // 一定周期確保のためのスリープ
        ddm_control_service_->intervalSleep(control_tick_ns, start);
        start = chrono::system_clock::now();
    }
}

void DefaultMainController::getControlMode(vector<double>& res) {
    spdlog::debug("GetControlMode");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(static_cast<double>(current_motor_states_->at(i).control_mode));
    }
}

void DefaultMainController::getJointCurrent(vector<double>& res) {
    spdlog::debug("GetJointCurrent");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).current);
    }
}

void DefaultMainController::calibrateJoint(vector<double>& ref) {
    spdlog::info("CalibrateJoint");
    auto joint_no = static_cast<int>(ref[0]);
    auto calibrate_angle = ref[1];
    if (joint_no < 0 || joint_no >= current_motor_states_->size()) {
        spdlog::error("Invalid joint_no: " + to_string(joint_no));
        return;
    }
    ddm_control_service_->calibrateJointWhenReady(joint_no, calibrate_angle, current_motor_states_->at(joint_no));
}

void DefaultMainController::calibrateJointFromMemory(vector<double>& ref) {
    spdlog::info("CalibrateJointFromMemory");
    auto joint_no = static_cast<int>(ref[0]);
    auto calibrate_angle = ref[1];
    auto memory_angle = ref[2];
    if (joint_no < 0 || joint_no >= current_motor_states_->size()) {
        spdlog::error("Invalid joint_no: " + to_string(joint_no));
        return;
    }
    ddm_control_service_->calibrateJointFromMemoryWhenReady(joint_no, calibrate_angle,
                                                            memory_angle, current_motor_states_->at(joint_no));
}

void DefaultMainController::alarmReset(vector<double>& joint_nos) {
    spdlog::info("AlarmReset");
    for (auto jn : joint_nos) {
        int joint_no = int(jn);
        if (joint_no < 0 || joint_no >= last_targets_.size()) {
            spdlog::error("Invalid joint_no: " + to_string(joint_no));
            return;
        }
        ddm_control_service_->alarmReset(joint_no, last_targets_[joint_no], current_motor_states_->at(joint_no));
    }
}

void DefaultMainController::getMotorStatus(vector<double>& res) {
    spdlog::debug("GetMotorStatus");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(static_cast<double>(motor_controller_->getMotorStatus(i, *current_motor_states_)));
    }
}

void DefaultMainController::forceStop(vector<double>& methods) {
    spdlog::info("ForceStop");
    auto control_tick_s = 1.0 / ControlFrequency;
    for (auto me : methods) {
        int method = int(me);
        ddm_control_service_->forceStop(method, *current_motor_states_, control_tick_s);
    }
}

void DefaultMainController::getJointSpeed(vector<double>& res) {
    spdlog::debug("GetJointSpeed");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).speed);
    }
}

void DefaultMainController::getJointTorque(vector<double>& res) {
    spdlog::debug("GetJointTorque");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).torque);
    }
}

void DefaultMainController::startLogging() {
    // カウンタクリアされてない場合：書き出しが終わってない場合なので注意
    // 現状では、仕様として、一つのログ書き出しが終わるまでは次のログ取得は禁止する
    if (!userlog_record_flag_ && userlog_count_ == 0) {
        spdlog::info("StartLogging");
        userlog_record_flag_ = true;
    } else {
        spdlog::error("Couldn't start logging");
    }
}

void DefaultMainController::stopLogging() {
    if (userlog_record_flag_) {
        spdlog::info("StopLogging");
        userlog_record_flag_ = false;
        // ファイル書き出しはスレッドで行う
        thread wthr(&DefaultMainController::writeUserLogToFile, this);
        wthr.detach();
    } else {
        spdlog::error("Logging is stopped");
    }
}

void DefaultMainController::clearLogs() {
    spdlog::info("ClearLogs");
    for (const auto& file : std::filesystem::directory_iterator(userlog_save_dir_)) {
        if( file.path().extension() == LOG_EXTENTION) {
            remove(file.path());
        }
    }
}

void DefaultMainController::getLogList(vector<string>& res) {
    spdlog::info("GetLogList");
    for (const auto& file : std::filesystem::directory_iterator(userlog_save_dir_)) {
        if( file.path().extension() == LOG_EXTENTION) {
            res.emplace_back(file.path());
        }
    }
}

void DefaultMainController::startTeaching(vector<double>& res) {
    spdlog::info("StartTeaching");
    if (teaching_id_ >= 0) {
        // 既に他の教示作業中のため失敗
        res.push_back(TEACHING_FAILED_ALREADY_TEACHING);
    }
    auto vacant_id = getVacantMotionId();
    if (vacant_id < 0) {
        // 既に容量一杯のため失敗
        res.push_back(TEACHING_FAILED_MOTION_FULL);
    } else {
        // teaching用のIDをセット
        teaching_id_ = vacant_id;
        res.push_back(TEACHING_OK);
    }
    teaching_flag_ = true;
}

void DefaultMainController::stopTeaching(vector<double>& res) {
    spdlog::info("StopTeaching");
    res.push_back(static_cast<double>(teaching_id_));
    finishTeaching();
}

void DefaultMainController::replayMotion(vector<double>& ref, vector<double>& res) {
    spdlog::info("ReplayMotion");
    auto joint_num = motor_controller_->getJointNum();
    int replay_id = static_cast<int>(ref.at(0));
    if (replay_id < 0 || replay_id >= max_teaching_motion_num_) {
        res.push_back(TEACHING_FAILED_NO_SUCH_ID);
    } else if(teaching_motions_.at(replay_id).empty()) {
        res.push_back(TEACHING_FAILED_NO_MOTION);
    } else {
        shared_ptr<vector<int> > masks;
        masks = make_shared<vector<int> >();
        copy(ref.begin() + 1, ref.begin() + 1 + joint_num, back_inserter(*masks));
        shared_ptr<deque<vector<double> > > ref_array = make_shared<deque<vector<double> > >();

        // add initial motion
        vector<double> goal_references = teaching_motions_.at(replay_id).at(0);
        auto ret = calcJointTrajectory(goal_references, DEFAULT_START_REPLAY_MOTION_TIME, Interpolation_MinJerk, masks, ref_array);
        if (!ret) {
            res.push_back(TEACHING_FAILED_INVALID_MOTION);
            return;
        }

        // recorded_motion
        for (auto& motion : teaching_motions_.at(replay_id)) {
            vector<double> joint_ref{};
            for (auto i = 0; i < joint_num; i++) {
                if (current_motor_states_->at(i).control_mode == ControlModePosition) {
                    // 位置制御の場合のみ有効
                    joint_ref.emplace_back(motion.at(i));
                } else {
                    joint_ref.emplace_back(0.0);
                }
            }
            ref_array->push_back(joint_ref);
        }
        ddm_control_service_->setJointTargetsWhenReady(ref_array, masks);
        res.push_back(TEACHING_OK);
    }
}

void DefaultMainController::getMotionList(vector<double>& res) {
    spdlog::info("GetMotionList");
    for (const auto& tm: teaching_motions_) {
        if (!tm.empty()) {
            auto id = &tm - &teaching_motions_[0];
            res.push_back(static_cast<double>(id));
        }
    }
}

void DefaultMainController::clearMotion(vector<double>& motion_id, vector<double>& res) {
    spdlog::info("ClearMotion");
    int clear_id = static_cast<int>(motion_id.at(0));
    if (clear_id < 0 || clear_id >= max_teaching_motion_num_) {
        res.push_back(TEACHING_FAILED_NO_SUCH_ID);
    } else {
        teaching_motions_.at(clear_id).clear();
        res.push_back(TEACHING_OK);
        string filepath = TEACHING_MOTION_DIR + string("/") + TEACHING_MOTION_BIN_BASENAME + to_string(clear_id) + TEACHING_MOTION_BIN_EXT;
        if (std::filesystem::exists(filepath)) {
            std::filesystem::remove(filepath);
        }
    }
}

void DefaultMainController::clearAllMotions() {
    spdlog::info("ClearAllMotions");
    for (auto& tm: teaching_motions_) {
        if (!tm.empty()) {
            tm.clear();
        }
    }
    for (const auto& file : std::filesystem::directory_iterator(TEACHING_MOTION_DIR)) {
        if( file.path().extension() == BIN_EXTENTION) {
            std::filesystem::remove(file.path());
        }
    }
}

int DefaultMainController::getVacantMotionId() {
    // 空いているIDの内もっとも小さいものを返す。全て埋まっている場合は-1を返す
    for (const auto& tm: teaching_motions_) {
        if (tm.empty()) {
            auto id = &tm - &teaching_motions_[0];
            return id;
        }
    }

    return -1;
}

void DefaultMainController::readMotionDataFromBin(int teaching_id) {
    string filepath = TEACHING_MOTION_DIR + string("/") + TEACHING_MOTION_BIN_BASENAME + to_string(teaching_id) + TEACHING_MOTION_BIN_EXT;
    std::ifstream fin(filepath, std::ios::in | std::ios::binary);
    string line;
    while(getline(fin, line)) {
        vector<string> split_str = Split(line, ',');
        vector<double> motdata{};
        for (auto i = 0; i < split_str.size(); ++i) {
            motdata.emplace_back(stod(split_str.at(i)));
        }
        teaching_motions_.at(teaching_id).push_back(motdata);
    }
    fin.close();
}

void DefaultMainController::writeMotionDataToBin(int teaching_id) {
    string filepath = TEACHING_MOTION_DIR + string("/") + TEACHING_MOTION_BIN_BASENAME + to_string(teaching_id) + TEACHING_MOTION_BIN_EXT;
    std::ofstream ofstr(filepath.c_str());
    auto joint_num = motor_controller_->getJointNum();
    for (const auto& mot : teaching_motions_.at(teaching_id)) {
        for(auto i = 0; i < joint_num; ++i){
            ofstr << mot.at(i) << ",";
        }
        ofstr << endl;
    }
    ofstr.close();
}

void DefaultMainController::finishTeaching() {
    writeMotionDataToBin(teaching_id_);
    teaching_flag_ = false;
    teaching_id_ = -1;
}

void DefaultMainController::controllerShutdown() {
    spdlog::info("ControllerShutdown");
    system("sudo shutdown -h now");
}

void DefaultMainController::getPong(vector<string>& res) {
    spdlog::info("GetPONG");
    res.emplace_back(string("PONG"));
}

void DefaultMainController::getAllSensorInfo(vector<double>& res) {
    spdlog::debug("GetAllSensorInfo");
    res.clear();
    auto joint_num = motor_controller_->getJointNum();
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).angle);
    }
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).current);
    }
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).speed);
    }
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(current_motor_states_->at(i).torque);
    }
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(static_cast<double>(current_motor_states_->at(i).control_mode));
    }
    for (int i = 0; i < joint_num; i++) {
        res.emplace_back(static_cast<double>(motor_controller_->getMotorStatus(i, *current_motor_states_)));
    }
}

void DefaultMainController::setZeroGMode(vector<double>& ref) {
    spdlog::debug("SetZeroGMode");
    auto joint_num = motor_controller_->getJointNum();

    if (joint_num < 3) {
        spdlog::error("SetZeroGMode can be used for manipulator with at least 3DOF");
        return;
    }

    if (ref.at(0) == 0) {
        motor_controller_->setZeroGMode(false);
    } else {
        motor_controller_->setZeroGMode(true);
    }
}
