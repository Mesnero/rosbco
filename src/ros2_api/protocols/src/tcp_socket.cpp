#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros2_api/protocols/tcp_socket.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <cerrno>
#include <cstring>
#include <thread>
#include <atomic>

namespace protocols
{
    TCPSocket::~TCPSocket()
    {
        stop();
    }

    void TCPSocket::start()
    {
        running_ = true;
        if (!processing_thread_.joinable())
        {
            processing_thread_ = std::thread([this]() { startReceiving(); });
        }
    }

    void TCPSocket::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "Stopped socket.");
        running_ = false;
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }

        if (socket_fd_ != -1)
        {
            close(socket_fd_);
        }
    }

    void TCPSocket::startReceiving()
    {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error creating socket");
            return;
        }

        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(port_);

        if (inet_pton(AF_INET, ip_address_.c_str(), &server_addr_.sin_addr) <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Invalid IP address: %s", ip_address_.c_str());
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "Binding TCP socket to %s:%d", ip_address_.c_str(), port_);
        if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_)) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error binding socket: %s", std::strerror(errno));
            return;
        }

        if (listen(socket_fd_, queue_size_connections_) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error listening on socket: %s", std::strerror(errno));
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "Ready for client to connect.");

        while (running_)
        {
            //TODO: Maybe safe IP and check for access control
            client_fd_ = accept(socket_fd_, nullptr, nullptr);
            if (client_fd_ == -1)
            {
                continue;
            }
            client_connected_ = true;
            RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "Client connected");
            handleClient();
            close(client_fd_);
            client_fd_ = -1;
        }
    }

    void TCPSocket::handleClient()
    {
        std::uint8_t buffer[max_message_size_];
        ssize_t bytes_read = 0;
        while (client_connected_ && running_)
        {
            memset(buffer, 0, sizeof(buffer));
            bytes_read = recv(client_fd_, buffer, sizeof(buffer), 0);
            if (bytes_read == -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error reading from client: %s", std::strerror(errno));
                break;
            }
            if (bytes_read == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "Client disconnected");
                client_connected_ = false;
                break;
            }
            callback_(buffer, bytes_read);
        }
    }

    void TCPSocket::sendToClient(const std::uint8_t* message, int length)
    {
        if (socket_fd_ != -1 && client_connected_)
        {
            if (write(client_fd_, message, length) == -1)
            {
                int err = errno; 
                RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error writing to client: %s (errno: %d)", std::strerror(err), err);
            }
        }
    }

    void TCPSocket::initialize(const YAML::Node& config)
    {
        // Set default IP address and port if not provided
        ip_address_ = "127.0.0.1";
        port_ = 8080;

        if (!config)
        {
            RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "No parameters given. Keeping defaults.");
        }

        if (config["ip_address"])
        {
            try
            {
                std::string ip = config["ip_address"].as<std::string>();
                if (!ip.empty())
                {
                    ip_address_ = ip;
                }
            }
            catch (YAML::TypedBadConversion<std::string>& ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error when setting ip_address.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "No ip_address found. Using default 127.0.0.1");
        }

        if (config["port"])
        {
            try
            {
                port_ = config["port"].as<int>();
            }
            catch (YAML::TypedBadConversion<int>& ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error when setting port.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "No port found. Using default 8080.");
        }

        if (config["queue_size"])
        {
            try
            {
                queue_size_connections_ = config["queue_size"].as<int>();
            }
            catch (YAML::TypedBadConversion<int>& ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error when setting queue_size.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "No queue_size found. Using default 1.");
        }

        if (config["max_message_size"])
        {
            try
            {
                max_message_size_ = config["max_message_size"].as<int>();
            }
            catch (YAML::TypedBadConversion<int>& ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error when setting max_message_size.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("TCPSocket"), "No max_message_size found. Using default 1024 bytes.");
        }
    }
} // namespace protocols

PLUGINLIB_EXPORT_CLASS(protocols::TCPSocket, protocol_base::CommunicationProtocol)
