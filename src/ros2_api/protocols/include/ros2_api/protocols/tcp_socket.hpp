#ifndef TCP_SOCKET_HPP
#define TCP_SOCKET_HPP

#include <yaml-cpp/yaml.h>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <cstdint>

namespace protocols
{
    class TCPSocket : public protocol_base::CommunicationProtocol
    {
    public: 
        ~TCPSocket();
        void sendToClient(const std::uint8_t *message, int length) override;
        void start() override;
        void initialize(const YAML::Node &config) override;
        void stop() override;
    private:
        void handleClient();
        void startReceiving();

        int socket_fd_;
        int client_fd_{-1};
        bool client_connected_{false};
        struct sockaddr_in server_addr_;
        std::string ip_address_{"127.0.0.1"};  // Default IP address for binding
        int port_{8080};                       // Default port
        int queue_size_connections_{1};
        int max_message_size_{1024};
        std::thread processing_thread_;
        std::atomic<bool> running_{false};
    };
} // namespace protocols

#endif // TCP_SOCKET_HPP
