#ifndef COMMUNICATION_PROTOCOL_HPP
#define COMMUNICATION_PROTOCOL_HPP

#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace protocol_base
{
    using CallbackType = std::function<void(const std::uint8_t *message, int length)>;
    class CommunicationProtocol
    {
    public:
        virtual ~CommunicationProtocol(){}

        virtual void initialize(const YAML::Node &config) = 0;
        virtual void sendToClient(const std::uint8_t *message, int length) = 0;
        virtual void start() = 0;
        virtual void stop() = 0;

        void setCallback(CallbackType callback) {
            callback_ = callback;
        }


    protected:
        CallbackType callback_;
        CommunicationProtocol() {}
    };

} // namespace protocol_base
#endif // COMMUNICATION_PROTOCOL_HPP