#ifndef COMMUNICATION_PROTOCOL_HPP
#define COMMUNICATION_PROTOCOL_HPP

#include <memory>
#include <thread>
#include <ros2_api/communication_manager.hpp>

namespace robot_api {

class CommunicationProtocol {
public:
    CommunicationProtocol(rclcpp::Logger logger);
    ~CommunicationProtocol();

    virtual void send(const char* message, int length) = 0;
    void start();
    void subscribe(CommunicationManager& manager);

protected:
    rclcpp::Logger logger_;
    void notifyManager(char* message, int length);

private:
    CommunicationManager* manager_ = nullptr;
    std::thread serverThread_;
    virtual void initialize() = 0;
};

} // namespace robot_api

#endif // COMMUNICATION_PROTOCOL_HPP
