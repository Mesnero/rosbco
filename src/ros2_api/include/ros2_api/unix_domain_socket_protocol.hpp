#ifndef UNIX_DOMAIN_SOCKET_PROTOCOL_HPP
#define UNIX_DOMAIN_SOCKET_PROTOCOL_HPP

#include <ros2_api/communication_protocol.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string>

namespace robot_api {

class UnixDomainSocketProtocol : public CommunicationProtocol {
public:
    UnixDomainSocketProtocol(rclcpp::Logger logger);
    ~UnixDomainSocketProtocol();

    void send(const char* message, int length) override;
    void initialize() override;

private:

    int socket_fd_;
    int client_fd_{-1};
    bool client_connected{false};
    struct sockaddr_un server_addr_;
    void handleClient();
};

} // namespace robot_api

#endif // UNIX_DOMAIN_SOCKET_PROTOCOL_HPP