# choice of which modification to perform
int8 				type

# list of nodes to be modified (added to, removed, or updated)
string[] 	to_modify

# optional: description of node to add to the tree
dhtt_msgs/Node 		add_node

# optional: file path and name of the subtree to add to the current tree
string			 	to_add

# optional: string description of the parameters to change (strings are of the form "param: val")
string[] 			params

# optional: type of node to mutate into (given by the types in the Node.msg file, or as a literal name in the case of behaviors)
string			 	mutate_type

# msg constants (self explanatory)
int8 				ADD=0
int8				ADD_FROM_FILE=1
int8 				REMOVE=2
int8 				PARAM_UPDATE=3
int8 				MUTATE=4
