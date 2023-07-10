#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "socket_handler.h"


using namespace hr4c_comm::socket_api;

SocketHandler::SocketHandler(string server_ip, int port)
        : server_ip_ {move(server_ip)},
          port_ {port} {

    // ソケットの作成
    sock_ = socket(AF_INET, SOCK_STREAM, 0);

    // 接続先指定用構造体の準備
    server_.sin_family = AF_INET;
    server_.sin_port = htons(port_);
    server_.sin_addr.s_addr = inet_addr(server_ip_.c_str());
}

int SocketHandler::openSocket() {
    lock_guard<mutex> lock(mutex_); // メンバ関数をロック

    // サーバに接続
    return connect(sock_, (struct sockaddr *) &server_, sizeof(server_));
}

int SocketHandler::closeSocket() {
    lock_guard<mutex> lock(mutex_); // メンバ関数をロック

    // socketのクローズ
    return close(sock_);
}

void SocketHandler::sendRecvCommand(const string& command, string& result, string& contents) {
    lock_guard<mutex> lock(mutex_); // メンバ関数をロック

    char buf[BUFFER_MAX];
    int send_count = send(sock_, command.c_str(), command.length(), 0);
    if (send_count == -1) {
        result = SND_ERROR_MSG;
        return;
    }

    // サーバからデータを受信
    int recv_size = recv(sock_, buf, BUFFER_MAX, 0);
    if (recv_size == -1) {
        result = RCV_ERROR_MSG;
        return;
    }
    result = SUCCESS_MSG;
    contents = string(buf);
}