# header containing the timestamp of the msg
std_msgs/Header			head

uint64 world_id # (unique id)
bool moveable
bool walkable
bool is_static

string object_type # Class.__name__ ie. Lettuce

geometry_msgs/Point location

string[] physical_state # ie. chopped and toasted

# optional: some food types have progress
int8 progress

# optional: for ContentObjects, ids of contained objects
# ros doesn't allow recursive msgs
uint64[] content_ids