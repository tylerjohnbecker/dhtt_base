# header containing the timestamp of the msg
std_msgs/Header			head

dhtt_msgs/CookingObject[] objects

# dict['str', tuple[tuple, float]] of key: object type to value: location, distance
string[] closest_objects_types
geometry_msgs/Point[] closest_objects_locations
float32[] closest_objects_distances

dhtt_msgs/CookingAgent[] agents

dhtt_msgs/CookingAction[] actions

uint64 ticks

# recipes allowed
dhtt_msgs/CookingRecipe[] recipes_allowed
dhtt_msgs/CookingRecipe[] recipes_completed
dhtt_msgs/CookingRecipe[] recipes_duds

#int8 last_reward
#int8 cumulative_reward