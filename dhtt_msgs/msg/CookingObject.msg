# header containing the timestamp of the msg
std_msgs/Header			head

int8 world_id # (unique id)
bool moveable
bool walkable

string object_type # Class.__name__ ie. Lettuce

geometry_msgs/Point location

string[] physical_state # ie. chopped and toasted

# optional: some food types have progress
int8 progress

# optional: for ContentObjects, ids of contained objects
# ros doesn't allow recursive msgs
int8[] content_ids