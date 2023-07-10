#ifndef HR4C_CORE_SOCKETSERVICE_HANDLER_H
#define HR4C_CORE_SOCKETSERVICE_HANDLER_H
#include <netinet/in.h>
#include <atomic>
#include <string>
#include "telegram_parser.h"
#include "hr4c_core.h"

namespace hr4c::socket_api {
    using namespace hr4c;
    using namespace telegram;

    constexpr auto BUFFER_MAX = 1024;
    class SocketServiceHandler
    {
    public:
        explicit SocketServiceHandler(int port, HR4CFacade& facade) : port_ {port}, facade_{facade} {};
        ~SocketServiceHandler();
        int initServer();
        int shutdownServer();
        void run();

        void processData();
    private:
        unique_ptr<TelegramParser> telegram_parser_;
        int port_;
        int sock_ = -1;
        atomic<bool> loop_flag { true };
        HR4CFacade& facade_;
        void executeCommand(string& command, vector<double>& args, string& output_str);
    };

} // namespace hr4c::socket_api

#endif //HR4C_CORE_SOCKETSERVICE_HANDLER_H
