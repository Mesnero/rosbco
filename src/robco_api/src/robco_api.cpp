#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <thread>
#include <vector>

//TODO: Add header file
static const char* SOCKET_PATH = "/tmp/robot_api.socket";
static const unsigned int QUEUE_SIZE_CONNECTIONS = 1;

class RobotAPINode : public rclcpp::Node
{
public:
  RobotAPINode() : Node("robot_api_node")
  {
    // Start the UNIX socket server in a separate thread
    server_thread_ = std::thread(&RobotAPINode::start_unix_socket_server, this);
  }

  ~RobotAPINode()
  {
    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
    close(server_fd_);
    unlink(SOCKET_PATH);
  }

private:
  int server_fd_;
  int client_fd_{ -1 };
  bool client_connected{ false };
  
  // TODO: Set based on config
  bool is_blocking_{ false };
  bool is_currently_blocked_{ false };
  /* TODO: Import from messages or create a parameter node?
  std::vector<StateInterfaces> state_interfaces_;
  std::vector<CmdInterfaces> cmd_interfaces_; 
  */


  std::thread server_thread_;

  void start_unix_socket_server()
  {
    server_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd_ == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket, stopping Node");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Socket created sucessfully.");

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    unlink(SOCKET_PATH);

    if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind socket, stopping Node");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Socket bound sucessfully.");

    if (listen(server_fd_, QUEUE_SIZE_CONNECTIONS) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket, stopping Node");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "API listening on %s", SOCKET_PATH);

    while (rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for client connection.");
      client_fd_ = accept(server_fd_, nullptr, nullptr);
      RCLCPP_INFO(this->get_logger(), "Client connection found.");
      if (client_fd_ == -1)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to accept client connection.");
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Client connected.");
      client_connected = true;
      handle_client();
      close(client_fd_);
      client_fd_ = -1;
      client_connected = false;
    }
  }

  void handle_client()
  {
    char buffer[1024];
    ssize_t bytes_read = 0;
    while (client_connected)
    {
      memset(buffer, 0, sizeof(buffer));
      // Read data from client
      // FOR TESTING NOW: JUST PRINT ALL DATA SENT TO THE LOGGER
      bytes_read = recv(client_fd_, buffer, sizeof(buffer), 0);
      if (bytes_read == -1)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data from client.");
        break;
      }
      if (bytes_read == 0)
      {
        RCLCPP_INFO(this->get_logger(), "Client disconnected.");
        break;
      }
      else if (bytes_read > 0)
      {
        RCLCPP_INFO(this->get_logger(), "Received data from client: %s", buffer);
      }
      // Parse data
      // Send data to validator (connection implemented as service)
      // Respond with status from validator
    }
  }

  void send_to_client()
  {
    if (client_connected)
    {
      // Send data to client
    }
  }

  void send_robot_config()
  {
    // Send robot configuration to client
  }

  void send_status()
  {
    // Send robot status to client if joint_states is updated (subscribe to it)
  }

  void send_error()
  {
    // Send error to client
  }

  void send_cmd_executed()
  {
    // Send command executed to client
  }
  

  void pass_to_validator()
  {
    // Send data to validator and interpret response
  }

  void handle_client_message()
  {
    // Handle client request
  }

  void handle_position_cmd()
  {
    // Handle position command
  }

  void handle_velocity_cmd()
  {
    // Handle velocity command
  }

  void handle_acceleration_cmd()
  {
    // Handle acceleration command
  }

  void handle_effort_cmd()
  {
    // Handle effort command
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotAPINode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
Client Methods:
isConnected()
getJointCount() -> Set when RobotConfiguration is called
getCmdInterfaces() -> Set when RobotConfiguration is called
getStateInterfaces() -> Set when RobotConfiguration is called
getConstraints() -> Set when RobotConfiguration is called
    - MAX_VEL -> array of doubles for each joint
    - MAX_EFF -> array of doubles for each joint
    - MAX_POS -> array of doubles for each joint
    - MAX_JERK -> array of doubles for each joint
    - MAX_ACC -> array of doubles for each joint
    - WORKSPACE -> 8 points
    - COLL_OBJECTS -> array of 8 points
sendVelocity() -> Server checks if it is a valid command & not blocked, then pass to validator
sendPosition() -> Server checks if it is a valid command & not blocked, then pass to validator
sendAcceleration() -> Server checks if it is a valid command & not blocked, then pass to validator
sendEffort() -> Server checks if it is a valid command & not blocked, then pass to validator
connect() -> Immediate answer with RobotConfiguration & returns a stream of messages
disconnect()
getStream() -> Rereturns the stream
*/