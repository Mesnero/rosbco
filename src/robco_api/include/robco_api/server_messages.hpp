// TODO: Namespace everything and tidy up.


const int MAX_JOINTS = 10;
const int MAX_MESSAGE_LENGTH = 256;
const int MAX_COLLISION_OBJECTS = 10;
const int MAX_STATE_INTERFACES = 5;
const int MAX_COMMAND_INTERFACES = 3;

struct __attribute__((packed)) Error {
	ServerMessageType type = ServerMessageType::ERROR;
	ErrorCode errorCode;
	int length_errorMsg;
	char errorMsg[MAX_MESSAGE_LENGTH]; 
};

// TODO: Calculate all values based of the state_interfaces? or just pass the ones defined through the API?
struct __attribute__((packed)) Status {
	ServerMessageType type = ServerMessageType::STATUS;
	int jointCount; //Only one length needed, since they are all the same size
	Point pos_abs_values[MAX_JOINTS];
	double pos_angle_values[MAX_JOINTS];
	double vel_values[MAX_JOINTS];	
	double acc_values[MAX_JOINTS];	
    double jerk_values[MAX_JOINTS];
};

// Only gets sent if API is set to block
struct __attribute__((packed)) CmdExecuted {
	ServerMessageType type = ServerMessageType::CMD_EXECUTED;
};

struct __attribute__((packed)) RobotConfig {
	ServerMessageType type = ServerMessageType::ROBOT_CONFIG;
	int jointCount;
	double max_pos[MAX_JOINTS]; 
	double max_vel[MAX_JOINTS]; 
	double max_acc[MAX_JOINTS]; 
	double max_eff[MAX_JOINTS]; 
	double max_jerk[MAX_JOINTS];
	Box workspace;
    int collisionObjectCount;
	Box collision_object[MAX_COLLISION_OBJECTS];
    StateInterface state_interfaces[MAX_STATE_INTERFACES];
    CommandInterface command_interfaces[MAX_COMMAND_INTERFACES]; 
};

struct __attribute__((packed)) ValidationFailed {
	ServerMessageType type = ServerMessageType::VALIDATION_FAILED;
	ValidationFailedType validationFailedType;
	int length_errorMsg;
	char errorMsg[MAX_MESSAGE_LENGTH]; 
};

struct __attribute__((packed)) Box {
	Point top_left_back;
	Point top_right_back;
	Point top_left_front;
	Point top_right_front;
	Point bottom_left_back;
	Point bottom_right_back;
	Point bottom_left_front;
	Point bottom_right_front;
};	

struct __attribute__((packed)) Point {
	double x;
	double y;
	double z;
};

enum ErrorCode {
	UNKNOWN_CMD_INTERFACE,
	UNKNOWN_MESSAGE_TYPE,
	UNEXPECTED_MSG_STRUCTURE,
	API_BLOCKED,
	ROBOT_DISCONNECTED_UNEXPECTEDLY,
	ROBOT_BREAKS // Sent if no message is received from client for a certain time period
};

// TODO: Which state interfaces to offer? Need to calculate all values anyway, so why not just send all?
enum StateInterface {
    POSITION_ABS,
	POSITION_ANGLE,
    VELOCITY,
    ACCELERATION,
    JERK
};

enum CommandInterface {
    POSITION,
    VELOCITY,
    EFFORT
};

enum ServerMessageType {
	ERROR, 
	STATUS, 
	CMD_EXECUTED, 
	ROBOT_CONFIG,
	VALIDATION_FAILED
};

enum ValidationFailedType {
	WORKSPACE_VIOLATION,
	SELF_COLLISION,
	COLLISION_WITH_COLLISION_OBJECT,
	POSITION_VIOLATION,
	JERK_VIOLATION,
	ACCELERATION_VIOLATION,
	VELOCITY_VIOLATION
};
