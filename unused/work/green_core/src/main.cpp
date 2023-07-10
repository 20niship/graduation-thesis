#include <sched.h>
#include <unistd.h>
#include <sys/mman.h>
#include <csignal>
#include <cstring>
#include <iostream>
#include "libhr4c_core.h"
#include <motor_controller/green_motor_control.h>


int main(int argc, char *argv[])
{
    using namespace std;
    using namespace hr4c;

    cout << "libhr4c_core: Version " << LibHR4CCoreVersion << endl;
    cout << "GreenCore: Version " << GreenCoreVersion << endl;

    // シャットダウンシグナルのハンドラ登録
    signal(SIGINT, [](int signum) {
        cout << endl << "Caught signal " << signum << ", shutting down Green core" << endl;
        MainController::Instance()->shutdown();
        exit(0);
    });

    // memoryのロック
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        cerr << "mlockall failed" << endl;
        exit(-2);
    }

    // 予めスタックフォルトを発生させることで、リアルタイムタスクにコンテクストスイッチが発生する際に
    // ページフォルトが起こるのを防止して、応答性を上げる
    auto max_safe_stack_size = 8 * 1024;
    unsigned char dummy[max_safe_stack_size];
    memset(dummy, 0, max_safe_stack_size);

    // コンフィグの読み込み
    const static YAML::Node Config {YAML::LoadFile("config.yaml")};

    // Green用のモータコントローラを生成
    auto joint_config = Config["joints"];
    shared_ptr<devices::GreenMotorControl> motor_controller_ = make_shared<devices::GreenMotorControl>(joint_config);

    return MainController::Instance()->run(Config, motor_controller_);
}
