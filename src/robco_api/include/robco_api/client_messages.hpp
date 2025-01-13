// TODO: Namespace everything, padding and tidy up.

const int MAX_JOINTS = 10;

struct PositionCmd {
	ClientMessageType type = ClientMessageType::POSITION;
	int length_pos;
	double pos_values[MAX_JOINTS];
};

struct VelocityCmd {
	ClientMessageType type = ClientMessageType::VELOCITY;
	int length_vel;
	double vel_values[MAX_JOINTS];
};

struct EffortCmd {
	ClientMessageType type = ClientMessageType::EFFORT;
	int length_eff;
	double effort_values[MAX_JOINTS];
};


enum ClientMessageType {
	POSITION, 
	VELOCITY, 
	EFFORT
};