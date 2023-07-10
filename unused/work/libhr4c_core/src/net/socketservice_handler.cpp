#include <ctime>
#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <spdlog/spdlog.h>
#include "net/socketservice_handler.h"


using json = nlohmann::json;
using namespace hr4c::socket_api;

SocketServiceHandler::~SocketServiceHandler() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

int SocketServiceHandler::initServer() {
    struct sockaddr_in addr{};

    // 電文パーサーの作成
    telegram_parser_ = make_unique<JSONTelegramParser>();

    // ソケット生成
    if( (sock_ = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) {
        spdlog::error("socket creation error");
        return -1;
    }

    // 待ち受け用IP・ポート番号設定
    addr.sin_family = AF_INET;
    addr.sin_port = htons( port_ );
    addr.sin_addr.s_addr = INADDR_ANY;

    // バインド
    if( bind( sock_, (struct sockaddr *)&addr, sizeof( addr ) ) < 0 ) {
        spdlog::error("bind error");
        return -1;
    }

    // ソケットへのリッスン
    if( listen( sock_, SOMAXCONN ) < 0 ) {
        spdlog::error("listen error");
        return -1;
    }

    return 0;
}

int SocketServiceHandler::shutdownServer() {
    loop_flag = false;
    return 0;
}

void SocketServiceHandler::run() {
    thread th(&SocketServiceHandler::processData, this);
    th.detach();
}

void SocketServiceHandler::executeCommand(string& command, vector<double>& args, string& output_str) {
    string response;
    vector<double> result;
    vector<string> result_str;

    if (command == CommandSetJointReference ) {
        facade_.setJointReference(args);
    } else if ( command == CommandSetJointTrajectory ) {
        facade_.setJointTrajectory(args);
    } else if ( command == CommandGetJointAngle ) {
        facade_.getJointAngle(result);
    } else if ( command == CommandWaitInterpolation ) {
        facade_.waitInterpolation();
    } else if ( command == CommandSetControlMode ) {
        facade_.setControlMode(args);
    } else if ( command == CommandServoOn ) {
        facade_.servoOn(args);
    } else if ( command == CommandServoAllOn ) {
        facade_.servoAllOn();
    } else if ( command == CommandServoOff ) {
        facade_.servoOff(args);
    } else if ( command == CommandServoAllOff ) {
        facade_.servoAllOff();
    } else if (command == CommandGetControlMode) {
        facade_.getControlMode(result);
    } else if (command == CommandGetJointCurrent) {
        facade_.getJointCurrent(result);
    } else if (command == CommandCalibrateJoint) {
        facade_.calibrateJoint(args);
    } else if (command == CommandCalibrateJointFromMemory) {
        facade_.calibrateJointFromMemory(args);
    } else if (command == CommandAlarmReset) {
        facade_.alarmReset(args);
    } else if (command == CommandGetMotorStatus) {
        facade_.getMotorStatus(result);
    } else if (command == CommandForceStop) {
        facade_.forceStop(args);
    } else if (command == CommandGetJointSpeed) {
        facade_.getJointSpeed(result);
    } else if (command == CommandGetJointTorque) {
        facade_.getJointTorque(result);
    } else if (command == CommandStartLogging) {
        facade_.startLogging();
    } else if (command == CommandStopLogging) {
        facade_.stopLogging();
    } else if (command == CommandClearLogs) {
        facade_.clearLogs();
    } else if (command == CommandGetLogList) {
        facade_.getLogList(result_str);
    } else if (command == CommandStartTeaching) {
        facade_.startTeaching(result);
    } else if (command == CommandStopTeaching) {
        facade_.stopTeaching(result);
    } else if (command == CommandReplayMotion) {
        facade_.replayMotion(args, result);
    } else if (command == CommandGetMotionList) {
        facade_.getMotionList(result);
    } else if (command == CommandClearMotion) {
        facade_.clearMotion(args, result);
    } else if (command == CommandClearAllMotions) {
        facade_.clearAllMotions();
    } else if(command == CommandControllerShutdown) {
        facade_.controllerShutdown();
    } else if(command == CommandGetPONG) {
        facade_.getPong(result_str);
    }  else if(command == CommandGetAllSensorInfo) {
        facade_.getAllSensorInfo(result);
    } else if(command == CommandSetZeroGMode) {
        facade_.setZeroGMode(args);
    } else {
        spdlog::error("Invalid command: " + command);
    }

    response = ResponseOK;
    telegram_parser_->createResponseString(command, response, result, result_str, output_str);
}

void SocketServiceHandler::processData() {
    int client_sock;
    ssize_t recv_size;
    char buf[BUFFER_MAX];
    socklen_t len = sizeof( struct sockaddr_in );
    struct sockaddr_in from_addr{};

    while (loop_flag) {
        // クライアントからのコネクト要求待ち
        if( ( client_sock = accept( sock_, (struct sockaddr *)&from_addr, &len ) ) < 0 ) {
            spdlog::error("accept error");
            continue;
        }
        spdlog::info("Connected");
        while (loop_flag) {
            // 受信バッファ初期化
            memset( buf, 0, sizeof( buf ) );
            recv_size = recv( client_sock, buf, sizeof( buf ), 0 );
            if ( recv_size <= 0) {
                spdlog::info("Disconnected");
                break;
            } else {
                // 入力文字列の解釈
                string input_str = string(buf);

                vector<Command> commands;
                telegram_parser_->parseTelegrams(input_str, commands);

                for (auto& command : commands) {
                    auto cmd = command.command;
                    auto args = command.args;
                    if (cmd != CommandParseError) {
                        // コマンドの実行と応答文字列の作成
                        string response_str;
                        executeCommand(cmd, args, response_str);
                        // JSON文字列チェック
                        try {
                            json jdata = json::parse(response_str);
                        }
                        catch (...) {
                            spdlog::error("wrong JSON Format");
                            continue;
                        }
                        // 文字列データの送信
                        try {
                            send(client_sock, response_str.c_str(), response_str.length(), 0);
                        }
                        catch (...) {
                            spdlog::error("socket connection closed");
                        }
                    }
                };
            }
        }
        close(client_sock);
    }
}
