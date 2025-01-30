#include <ros2_api/unix_domain_socket_protocol.hpp>
#include <iostream>
#include <cstring>

namespace robot_api
{
    static const char *SOCKET_PATH = "/tmp/robot_api.socket";
    static const unsigned int QUEUE_SIZE_CONNECTIONS = 1;
    static const unsigned int MAX_MESSAGE_SIZE = 1024;

    UnixDomainSocketProtocol::UnixDomainSocketProtocol(rclcpp::Logger logger) : CommunicationProtocol(logger) {}

    UnixDomainSocketProtocol::~UnixDomainSocketProtocol()
    {
        if (socket_fd_ != -1)
        {
            close(socket_fd_);
        }
        unlink(SOCKET_PATH);
    }

    void UnixDomainSocketProtocol::initialize()
    {
        socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
        if (socket_fd_ == -1)
        {
            RCLCPP_ERROR(logger_, "Error creating socket");
            return;
        }

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
        unlink(SOCKET_PATH);

        if (bind(socket_fd_, (sockaddr *)&addr, sizeof(addr)) == -1)
        {
            RCLCPP_ERROR(logger_, "Error binding socket");
            return;
        }
        if (listen(socket_fd_, QUEUE_SIZE_CONNECTIONS) == -1)
        {
            RCLCPP_ERROR(logger_, "Error listening on socket");
            return;
        }

        while (true)
        {
            client_fd_ = accept(socket_fd_, nullptr, nullptr);
            if (client_fd_ == -1)
            {
                continue;
            }
            client_connected = true;
            RCLCPP_INFO(logger_, "Client connected");
            UnixDomainSocketProtocol::handleClient();
            close(client_fd_);
            client_fd_ = -1;
        }
    }

    void UnixDomainSocketProtocol::handleClient()
    {
        char buffer[MAX_MESSAGE_SIZE];
        ssize_t bytes_read = 0;
        while (client_connected)
        {
            memset(buffer, 0, sizeof(buffer));
            bytes_read = recv(client_fd_, buffer, sizeof(buffer), 0);
            if (bytes_read == -1)
            {
                RCLCPP_ERROR(logger_, "Error reading from client");
                break;
            }
            if (bytes_read == 0)
            {
                RCLCPP_INFO(logger_, "Client disconnected");
                client_connected = false;
                break;
            }
            RCLCPP_INFO(logger_, "Message Received");

            UnixDomainSocketProtocol::notifyManager(buffer, bytes_read);
        }
    }

    void UnixDomainSocketProtocol::send(const char *message, int length)
    {
        if (socket_fd_ != -1 && client_connected)
        {
            if (write(socket_fd_, message, length) == -1)
            {
                RCLCPP_ERROR(logger_, "Error writing to client");
            }
        }
    }

} // namespace robot_api