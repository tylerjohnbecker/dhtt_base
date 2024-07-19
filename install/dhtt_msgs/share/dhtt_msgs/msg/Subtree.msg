# this is the total list of nodes in the subtree contained in the message
dhtt_msgs/Node[]	tree_nodes

# this is a collection of information and statistics about the tree
int8				tree_status
int32				max_tree_depth
int32				max_tree_width

# completion percent is the total number of nodes completed in the tree divided by the total number of nodes
float32				task_completion_percent