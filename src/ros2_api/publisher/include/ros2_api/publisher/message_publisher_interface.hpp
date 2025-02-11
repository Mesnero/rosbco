#ifndef MESSAGE_PUBLISHER_INTERFACE_HPP
#define MESSAGE_PUBLISHER_INTERFACE_HPP
#include <string>

class IMessagePublisher {
public:
  virtual ~IMessagePublisher() = default;
  
  // Publish a message (using a void pointer for type erasure).
  virtual void publish(const void* msg) = 0;
};

#endif // MESSAGE_PUBLISHER_INTERFACE_HPP