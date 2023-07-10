#include <filesystem>
#include <fstream>
#include <iostream>
#include <libssh2.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "hr4c_comm.h"


using namespace  hr4c_comm;


int HR4CSocketClient::start()
{
    auto ret = -1;

    // 1プロセス内での多重オープン禁止
    if (!connected_) {
        initControllerSocketHandler(ip_addr_, controller_port_);
        initCommandService();

        ret = controller_socket_handler_->openSocket();
        connected_ = ret >= 0;
    }

    return ret >= 0;
}

int HR4CSocketClient::stop()
{
    auto ret = -1;

    // 多重クローズ禁止
    if (connected_) {
        ret = controller_socket_handler_->closeSocket();
        connected_ = ret < 0;
    }

    return ret >= 0;
}

void HR4CSocketClient::controllerSetGetCommand(const string& command, vector<double> &ref, vector<double> &res,
                                               vector<string> &res_str, bool check_response) {
    if (connected_) {
        string command_str, result_msg, contents_str;
        command_service_->createCommandString(command, ref, command_str);
        controller_socket_handler_->sendRecvCommand(command_str, result_msg, contents_str);

        if (check_response && result_msg == SUCCESS_MSG) {
            string resp_command, response;
            command_service_->parseCommandResponse(contents_str, resp_command, response, res, res_str);
        }
    }
}

void HR4CSocketClient::controllerSetCommand(const string& command, vector<double> &ref) {
    vector<double> dummy;
    vector<string> dummy_str;
    controllerSetGetCommand(command, ref, dummy, dummy_str, false);
}

void HR4CSocketClient::controllerGetCommand(const string& command, vector<double> &res, vector<string> &res_str) {
    vector<double> dummy;
    vector<string> dummy_str;
    controllerSetGetCommand(command, dummy, res, res_str, true);
}

int HR4CSocketClient::scpFile(const string &filepath, const int direction) {
    int sock;
    int rc;
    struct sockaddr_in sin;
    string filename;
    string remote_filepath;
    unsigned long hostaddr = inet_addr(ip_addr_.c_str());
    LIBSSH2_SESSION *session;
    LIBSSH2_CHANNEL *channel;
    libssh2_struct_stat fileinfo;
    libssh2_struct_stat_size got = 0;
    ofstream fout;
    FILE* local;
    char mem[1024];
    char *ptr;
    auto nread = 0;

    // libssh2 initialization
    rc = libssh2_init(0);
    if(rc) {
        std::cerr << "libssh2 initialization failed (" << rc <<  ")" << std::endl;
        return -1;
    }

    // make socket for ssh
    sock = socket(AF_INET, SOCK_STREAM, 0);
    sin.sin_family = AF_INET;
    sin.sin_port = htons(22);
    sin.sin_addr.s_addr = hostaddr;
    if(connect(sock, (struct sockaddr*)(&sin),
               sizeof(struct sockaddr_in)) != 0) {
        std::cerr << "failed to connect!" << std::endl;
        return -1;
    }

    // get filename
    filesystem::path p = filepath;
    filename = p.filename();

    // Create a session instance
    session = libssh2_session_init();
    if(!session) {
        std::cerr << "failed to create session!" << std::endl;
        return -1;
    }

    // establishing SSH session
    rc = libssh2_session_handshake(session, sock);
    if(rc) {
        std::cerr << "Failure establishing SSH session: " << rc << std::endl;
        return -1;
    }

    auto controller_pass = "";
    if (model_ == ROBOT_MODEL_GREEN) {
        controller_pass = GREEN_PASS;
    } else if (model_ == ROBOT_MODEL_AMBER) {
        controller_pass = AMBER_PASS;
    } else {
        // defaultはAmber
        controller_pass = AMBER_PASS;
    }
    if(libssh2_userauth_password(session, CONTROLLER_USER, controller_pass)) {
        std::cerr << "Authentication by password failed." << std::endl;
        return -1;
    }

    switch(direction) {
        case SCP_RECV:
            // Request a file via SCP
            channel = libssh2_scp_recv2(session, filepath.c_str(), &fileinfo);
            if(!channel) {
                std::cerr << "Unable to open a session: " << libssh2_session_last_errno(session) << std::endl;
                return -1;
            }
            // open file
            fout.open(filename, ios::out | ios::trunc);
            if (!fout) {
                std::cerr << "Couldn't open file: " << filename << std::endl;
                return -1;
            }

            // read data from channel & write it to file
            while(got < fileinfo.st_size) {
                char mem[1024];
                int amount = sizeof(mem);

                if((fileinfo.st_size - got) < amount) {
                    amount = (int)(fileinfo.st_size - got);
                }

                rc = libssh2_channel_read(channel, mem, amount);
                if(rc > 0) {
                    fout.write(mem, rc);
                } else if(rc < 0) {
                    std::cerr << "libssh2_channel_read() failed: " << rc << std::endl;
                    break;
                }
                got += rc;
            }
            break;
        case SCP_SEND:
            local = fopen(filepath.c_str(), "rb");
            if(!local) {
                std::cerr << "Can't open local file: " << filepath << std::endl;
                return -1;
            }

            stat(filepath.c_str(), &fileinfo);

            // Send a file via scp. The mode parameter must only have permissions!
            // Here, saved directory is fixed (this is only used for update controller software)
            remote_filepath = string(BIN_SAVE_DIR) + "/" + filename + ".new";
            channel = libssh2_scp_send(session, remote_filepath.c_str(), fileinfo.st_mode & 0777,
                                       (unsigned long)fileinfo.st_size);

            if(!channel) {
                std::cerr << "Unable to open a session" << libssh2_session_last_errno(session) << std::endl;
                return -1;
            }

            // SCP session waiting to send file
            do {
                nread = fread(mem, 1, sizeof(mem), local);
                if(nread <= 0) {
                    // end of file
                    break;
                }
                ptr = mem;

                do {
                    // write the same data over and over, until error or completion
                    rc = libssh2_channel_write(channel, ptr, nread);

                    if(rc < 0) {
                        std::cerr << "SCP send error: " << rc << std::endl;
                        break;
                    }
                    else {
                        // rc indicates how many bytes were written this time
                        ptr += rc;
                        nread -= rc;
                    }
                } while(nread);
            } while(true);

            // Sending EOF
            libssh2_channel_send_eof(channel);
            break;
        default:
            // do nothing
            std::cerr << "Unsuported MODE: " << direction << std::endl;
            return -1;
    }

    // release resources
    libssh2_channel_free(channel);
    channel = nullptr;
    libssh2_session_disconnect(session,"Normal Shutdown, Thank you for playing");
    libssh2_session_free(session);
    close(sock);
    libssh2_exit();

    return 0;
}

void HR4CSocketClient::setJointReference(vector<double> &ref) {
    controllerSetCommand(CommandSetJointReference, ref);
}

void HR4CSocketClient::setJointTrajectory(vector<double> &ref) {
    controllerSetCommand(CommandSetJointTrajectory, ref);
}

void HR4CSocketClient::setControlMode(vector<double> &ref) {
    controllerSetCommand(CommandSetControlMode, ref);
}

void HR4CSocketClient::startLogging() {
    vector<double> dummy;
    controllerSetCommand(CommandStartLogging, dummy);
}

void HR4CSocketClient::stopLogging() {
    vector<double> dummy;
    controllerSetCommand(CommandStopLogging, dummy);
}

void HR4CSocketClient::clearLogs() {
    vector<double> dummy;
    controllerSetCommand(CommandClearLogs, dummy);
}

void HR4CSocketClient::getLogList(vector<string> &res_str) {
    vector<double> dummy;
    vector<string> paths;
    controllerGetCommand(CommandGetLogList, dummy, paths);
    for (auto &filepath : paths) {
        filesystem::path p = filepath;
        res_str.push_back(p.filename());
    }
}

int HR4CSocketClient::getLog(string &filename) {
    vector<double> dummy;
    vector<string> paths;
    string copy_filepath;

    controllerGetCommand(CommandGetLogList, dummy, paths);
    for (auto &filepath : paths) {
        filesystem::path p = filepath;
        if (p.filename() == filename) {
            copy_filepath = filepath;
            break;
        }
    }

    if (!copy_filepath.empty()) {
        return scpFile(copy_filepath, SCP_RECV);
    } else {
        return -1;
    };
}

void HR4CSocketClient::servoOn(vector<double> &joint_nos) {
    controllerSetCommand(CommandServoOn, joint_nos);
}

void HR4CSocketClient::servoAllOn() {
    vector<double> dummy;
    controllerSetCommand(CommandServoAllOn, dummy);
}

void HR4CSocketClient::servoOff(vector<double> &joint_nos) {
    controllerSetCommand(CommandServoOff, joint_nos);
}

void HR4CSocketClient::servoAllOff() {
    vector<double> dummy;
    controllerSetCommand(CommandServoAllOff, dummy);
}

void HR4CSocketClient::getJointAngle(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetJointAngle, res, dummy_str);
}

void HR4CSocketClient::getControlMode(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetControlMode, res, dummy_str);
}

void HR4CSocketClient::getJointCurrent(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetJointCurrent, res, dummy_str);
}

void HR4CSocketClient::waitInterpolation() {
    vector<string> dummy_str;
    vector<double> dummy_res;
    controllerGetCommand(CommandWaitInterpolation, dummy_res, dummy_str);
}

int HR4CSocketClient::initControllerSocketHandler(string &ip_addr, int port)
{
    controller_socket_handler_ = make_unique<SocketHandler>(ip_addr, port);
    return 0;
}

int HR4CSocketClient::initCommandService()
{
    command_service_ = make_unique<JSONCommandService>();
    return 0;
}

void HR4CSocketClient::calibrateJoint(vector<double> &ref) {
    controllerSetCommand(CommandCalibrateJoint,ref);
}

void HR4CSocketClient::calibrateJointFromMemory(vector<double> &ref) {
    controllerSetCommand(CommandCalibrateJointFromMemory,ref);
}

void HR4CSocketClient::alarmReset(vector<double> &joint_nos) {
    controllerSetCommand(CommandAlarmReset, joint_nos);
}

void HR4CSocketClient::getMotorStatus(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetMotorStatus, res, dummy_str);
}

void HR4CSocketClient::forceStop(vector<double> &methods) {
    controllerSetCommand(CommandForceStop, methods);
}

void HR4CSocketClient::getJointSpeed(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetJointSpeed, res, dummy_str);
}

void HR4CSocketClient::getJointTorque(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetJointTorque, res, dummy_str);
}

void HR4CSocketClient::startTeaching(vector<double> &res) {
    vector<double> dummy;
    vector<string> dummy_str;
    controllerSetGetCommand(CommandStartTeaching, dummy, res, dummy_str, true);
}

void HR4CSocketClient::stopTeaching(vector<double> &res) {
    vector<double> dummy;
    vector<string> dummy_str;
    controllerSetGetCommand(CommandStopTeaching, dummy, res, dummy_str, true);
}

void HR4CSocketClient::replayMotion(vector<double> &ref, vector<double> &res) {
    vector<string> dummy_str;
    controllerSetGetCommand(CommandReplayMotion, ref, res, dummy_str, true);
}

void HR4CSocketClient::getMotionList(vector<double> &res) {
    vector<double> dummy;
    vector<string> dummy_str;
    controllerSetGetCommand(CommandGetMotionList, dummy, res, dummy_str, true);
}

void HR4CSocketClient::clearMotion(vector<double> &ref, vector<double> &res) {
    vector<string> dummy_str;
    controllerSetGetCommand(CommandClearMotion, ref, res, dummy_str, true);
}

void HR4CSocketClient::clearAllMotions() {
    vector<double> dummy;
    controllerSetCommand(CommandClearAllMotions, dummy);
}

void HR4CSocketClient::controllerShutdown() {
    vector<double> dummy;
    controllerSetCommand(CommandControllerShutdown, dummy);
}

bool HR4CSocketClient::ping() {
    vector<double> dummy_res;
    vector<string> strs;
    controllerGetCommand(CommandGetPONG, dummy_res, strs);

    if (!strs.empty()) {
        if (strs.at(0) == string("PONG")) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

int HR4CSocketClient::updateController(string &binary_dir) {
    auto exe_path = binary_dir + "/" + "hr4c_core_rt";
    auto config_path = binary_dir + "/" + "config.yaml";

    if (!filesystem::exists(binary_dir)) {
        std::cerr << "No such directory: " << binary_dir << std::endl;
        return -1;
    }
    if (!filesystem::exists(exe_path)) {
        std::cerr << "hr4c_core could not be found in " << binary_dir << std::endl;
        return -1;
    }
    if (!filesystem::exists(config_path)) {
        std::cerr << "config.yaml could not be found in " << binary_dir << std::endl;
        return -1;
    }

    if(scpFile(exe_path, SCP_SEND) < 0) {
        return -1;
    }
    if(scpFile(config_path, SCP_SEND) < 0) {
        return -1;
    };

    std::cerr << "Controller software is successfully updated!" << std::endl;
    return 0;
}

void HR4CSocketClient::getAllSensorInfo(vector<double> &res) {
    vector<string> dummy_str;
    controllerGetCommand(CommandGetAllSensorInfo, res, dummy_str);
}

bool HR4CSocketClient::checkIdentity(const string& server_ip, int port) {
    if (server_ip == ip_addr_ && port == controller_port_) {
        return true;
    } else {
        return false;
    }
}

void HR4CSocketClient::enableZeroGMode(vector<double> &ref) {
    controllerSetCommand(CommandSetZeroGMode, ref);
}
