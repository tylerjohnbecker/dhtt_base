# header containing the timestamp of the msg
std_msgs/Header			head

# Agent is a subclass of Object, so use this to refer to those members
dhtt_msgs/CookingObject object_members

int8 orientation

# what the agent is holding
dhtt_msgs/CookingObject[] holding