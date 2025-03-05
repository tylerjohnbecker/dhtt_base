# header containing the timestamp of the msg
std_msgs/Header			head

# the planer that should be performing this action, usually "player_1"
string player_name

# type of action, including movement, interaction and move_to
int8 action_type

# optional: params for high-level actions
# move_to takes a comma-separated x-y string (ie. '1, 3') which gets parsed later
string params


# CONSTANTS
string DEFAULT_AGENT_ID = "agent-1"
string DEFAULT_PLAYER_NAME = "player_0"

int8 WALK_UP = 4
int8 WALK_DOWN = 3
int8 WALK_RIGHT = 2
int8 WALK_LEFT = 1

int8 NO_OP = 0

# automatically select hand
int8 INTERACT_PRIMARY = 5
int8 INTERACT_PICK_UP_SPECIAL = 6
int8 EXECUTE_ACTION = 7

# enforce handedness
int8 INTERACT_PRIMARY_ARM1 = 15
int8 INTERACT_PICK_UP_SPECIAL_ARM1 = 16
int8 EXECUTE_ACTION_ARM1 = 17
int8 INTERACT_PRIMARY_ARM2 = 25
int8 INTERACT_PICK_UP_SPECIAL_ARM2 = 26
int8 EXECUTE_ACTION_ARM2 = 27

# high-level command
int8 MOVE_TO = 100