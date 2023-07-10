#ifndef SOCKET_HANDLER_H
#define SOCKET_HANDLER_H
#include <netinet/in.h>
#include <atomic>
#include <mutex>
#include <string>

namespace hr4c_comm::socket_api {
    using namespace std;

    constexpr auto BUFFER_MAX = 1024;
    constexpr auto SUCCESS_MSG = "Success";
    constexpr auto SND_ERROR_MSG = "SendError";
    constexpr auto RCV_ERROR_MSG = "ReceiveError";
    constexpr auto NOT_CONNECTED_MSG = "NotConnected";
    class SocketHandler
    {
    public:
        SocketHandler(string server_ip, int port);
        ~SocketHandler() = default;
        int openSocket();
        int closeSocket();
        void sendRecvCommand(const string& command, string& results, string& contents);
    private:
        mutex mutex_;
        string server_ip_;
        int port_;
        struct sockaddr_in server_{};
        int sock_;
    };

} // namespace hr4c::socket_api

#endif // SOCKET_HANDLER_H
