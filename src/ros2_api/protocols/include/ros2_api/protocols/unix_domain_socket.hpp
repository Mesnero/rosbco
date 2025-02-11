#ifndef UNIX_DOMAIN_SOCKET_HPP
#define UNIX_DOMAIN_SOCKET_HPP

#include <yaml-cpp/yaml.h>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace protocols
{
    class UnixDomainSocket : public protocol_base::CommunicationProtocol
    {
        public: 
            ~UnixDomainSocket();
            void sendToClient(const std::uint8_t *message, int length);
            void start();
            void initialize(const YAML::Node &config);
            void stop();
        private:
            void handleClient();
            void startReceiving();

            int socket_fd_;
            int client_fd_{-1};
            bool client_connected_{false};
            struct sockaddr_un server_addr_;
            std::string socket_path_{"/tmp/ros2_api.socket"};
            int queue_size_connections_{1};
            int max_message_size_{1024};
            std::thread processing_thread_;
            std::atomic<bool> running_{false};
    };
} // namespace protocols
#endif // UNIX_DOMAIN_SOCKET_HPP


// TODO: MAYBE CHANGE CALLBACK STUFF