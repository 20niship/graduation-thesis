#ifndef HR4C_CORE_UDP_CLIENT_H
#define HR4C_CORE_UDP_CLIENT_H
#include <string>
#include <mutex>
#include <utility>
#include <spdlog/spdlog.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

namespace hr4c::udp_client {
    class UdpClient {
    public:
        UdpClient(std::string ip, int send_port, int recv_port) : ip_{std::move(ip)},
                                                                  send_port_{send_port}, recv_port_{recv_port} {}
        ~UdpClient() noexcept {
            auto ret = doClose();
            if (ret > 0) {
                spdlog::error("Failed to close socket.");
            }
        }

        void open() {
            std::lock_guard g{mtx_};
            if (sock_ > 0) {
                throw std::runtime_error("UDP socket is already open.");
            }

            sock_ = socket(AF_INET, SOCK_DGRAM, 0);

            saddr_.sin_family = AF_INET;
            saddr_.sin_addr.s_addr = inet_addr(ip_.c_str());
            saddr_.sin_port = htons(send_port_);

            raddr_.sin_family = AF_INET;
            raddr_.sin_addr.s_addr = INADDR_ANY;
            raddr_.sin_port = htons(recv_port_);
            bind(sock_, (struct sockaddr *)&raddr_, sizeof(raddr_));

            spdlog::info("Opened UDP socket: address {}, send_port {}, recv_port {}, socket {}",
                         ip_, send_port_, recv_port_,sock_);
        }


        int sendAllData(const uint8_t *data, const size_t send_len) const {
            std::lock_guard g{mtx_};
            if (!isOpen()) return 0;

            int sent = 0;
            while (sent < send_len) {
                sent += sendData(data + sent, send_len - sent);
            }

            return sent;
        }

        int receive(uint8_t *buf, int buf_size) const {
            std::lock_guard g{mtx_};
            if (!isOpen()) return 0;

            // 単にrecvを呼び出す。
            return recv(sock_, buf, buf_size, 0);
        }

        int receiveAllData(uint8_t *buf, const size_t receive_len) const {
            std::lock_guard g{mtx_};
            if (!isOpen()) return 0;

            auto total_received = 0;

            while (total_received < receive_len) {
                auto received = recv(sock_, buf + total_received, receive_len - total_received, 0);
                total_received += received;
            }

            return total_received;
        }

        bool isOpen() const {
            return sock_ >= 0;
        }

    private:
        const std::string ip_;
        int send_port_, recv_port_;
        mutable std::mutex mtx_;
        int sock_{-1};
        struct sockaddr_in saddr_{};
        struct sockaddr_in raddr_{};

        [[nodiscard]]
        ssize_t sendData(const uint8_t *data, int send_len) const {

            auto sent = sendto(sock_, data, send_len, 0,
                               reinterpret_cast<const sockaddr *>(&saddr_), sizeof(saddr_));
            if (sent == -1) {
                spdlog::error("Error sending data. error no = {}", errno);
            }

            return sent;
        }

        int doClose() {
            std::lock_guard g{mtx_};

            if (sock_ != -1) {
                if (close(sock_) == 0) {
                    sock_ = -1;
                }
            }
            return sock_;
        }
    };
}
#endif //HR4C_CORE_UDP_CLIENT_H
