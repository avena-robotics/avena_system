#define GUI_REFRESH_DELAY 100    //ms
#define MAX_HEARTBEAT_DELAY 1000 //ms

#define MILLI_NANO_MULTIPLIER 1000000

#define COMMAND_COL 3
#define NODE_NAME_COL 0
#define NODE_LAST_HEARTBEAT_COL 1
#define NODE_STATUS 2
#define START_ACTION_COMMAND "START"
#define STOP_ACTION_COMMAND "STOP"
#define PICK_PLACE_ACTION_SERVER_NAME "logic_server"

#define START_LAUNCH_FILE "heartbeats_test_launch.py"

#define BT_WARNING_DURATION 5

#define DANGER_TOOL_PUBLISHING_FREQ 200

#define PID_FILE_NAME "pid.dat"

enum class ControlCommands{
    START=1,
    STOP=0,
    RESUME=2,
    PAUSE=3
};



