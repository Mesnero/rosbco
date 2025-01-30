#include <ros2_api/communication_protocol.hpp>
#include <ros2_api/communication_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_api {

    CommunicationProtocol::CommunicationProtocol(rclcpp::Logger logger) : logger_(logger) {}

    CommunicationProtocol::~CommunicationProtocol() {
        if (serverThread_.joinable()) {
            serverThread_.join();
        }
    }

    //TODO: Do it this way or set with constructor?
    void CommunicationProtocol::subscribe(CommunicationManager& manager) {
        manager_ = &manager;
    }

    void CommunicationProtocol::notifyManager(char* message, int length) {
        if (manager_) {
            manager_->receiveMessage(message, length);
        }
    }

    void CommunicationProtocol::start() {
        serverThread_ = std::thread([this] {
            initialize();
        });
    }

} // namespace robot_api
