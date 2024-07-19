# header containing the timestamp of the msg
std_msgs/Header			head

# this is the topic name of the node in the tree
string 					node_name

# this is the index of the parent in the list from the dhtt_subtree message (-1 if this node is the root) 
int32					parent
string					parent_name

# this is a list of the indices of this nodes children from the dhtt_subtree message (empty if this node is a leaf)
int32[] 				children
string[]				child_name

# this is the list of params for the behavior
string[]				params

# this is the type of each node
int8 					type
string					plugin_name

# the resources that the node has been given access to
dhtt_msgs/Resource[]		owned_resources

# this is an indicator of the status of the node
dhtt_msgs/NodeStatus	node_status

# node type declarations (for specific behavior types use the common name parameter as well)
int8 					ROOT=0
int8 					AND=1
int8 					THEN=2
int8 					OR=3
int8 					BEHAVIOR=4