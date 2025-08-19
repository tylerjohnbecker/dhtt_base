# header containing the timestamp of the msg
std_msgs/Header			head

string recipe_name

# objects needed to fulfill this recipe, including their states (ie. chopped)
dhtt_cooking_msgs/CookingObject[] required_objects