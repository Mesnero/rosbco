#ifndef MESSAGE_PUBLISHER_HPP
#define MESSAGE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <ros2_api/publisher/message_publisher_interface.hpp>

template<typename MsgType>
class TypedPublisher : public IMessagePublisher {
public:
  TypedPublisher(const rclcpp::Node::SharedPtr& node,
                 const std::string& topic,
                 size_t queue_size)
  {
    publisher_ = node->create_publisher<MsgType>(topic, queue_size);
  }

  void publish(const void* msg) override {
    publisher_->publish(*static_cast<const MsgType*>(msg));
  }

private:
  typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
};

#endif // MESSAGE_PUBLISHER_HPP