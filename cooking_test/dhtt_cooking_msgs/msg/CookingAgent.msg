# header containing the timestamp of the msg
std_msgs/Header			head

# Agent is a subclass of Object, so use this to refer to those members
dhtt_cooking_msgs/CookingObject object_members

# direction the agent is facing
int8 orientation

# what the agent is holding
dhtt_cooking_msgs/CookingObject[] holding

int8 WEST=1
int8 EAST=2
int8 SOUTH=3
int8 NORTH=4